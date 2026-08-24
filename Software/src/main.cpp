#include <Arduino.h>
#include <ACAN_T4.h>

#include "AOCSControllerTelemetry.h"
#include "OBCConnection.h"
#include "ReactionWheel.h"
#include "ThrusterControl.h"

// Debug options
#define AOCS_SPI_DEBUG false
#define AOCS_ISOLATION_MODE false
#define THRUSTER_CONTROL_DEBUG false

// Pin assignments
constexpr uint8_t CAN_SILENT_MODE_PIN = 21;
// Thruster pin assignments -- per AOCS Hardware Control Board schematic
// (TCTRL1-4 net labels): Thruster1=D1, Thruster2=D4, Thruster3=D29, Thruster4=D22
constexpr uint8_t THRUSTER_PINS[ThrusterControl::kThrusterCount] = {1, 4, 29, 22};

#if AOCS_ISOLATION_MODE
  #include "MockOBCConnection.h"
  MockOBCConnection obcConnection = MockOBCConnection();
#else
  // The connection to the OBC (SPI)
  OBCConnection obcConnection = OBCConnection(AOCS_SPI_DEBUG);
#endif

// CAN bus settings: 1 Mbps arbitration and 5 Mbps data rate for CAN-FD bus speed
ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x5);
// The reaction wheel controller
ReactionWheel wheelController(CAN_SILENT_MODE_PIN, ACAN_T4::can3, canSettings, 1);
ThrusterControl thrusterController(THRUSTER_PINS);

// Current system telemetry
AOCSControllerTelemetry currentTelemetry;

// UTILITY METHODS
namespace AirSat {
  String reactionWheelModeToString(ReactionWheel::RWMode mode) {
    switch (mode) {
      case ReactionWheel::RWMode::kStopped:
        return "Stopped";
      case ReactionWheel::RWMode::kRunning:
        return "Running";
      case ReactionWheel::RWMode::kFault:
        return "Fault";
      default:
        return "Unknown";
    }
  }

  String reactionWheelFaultToString(ReactionWheel::RWFault fault) {
    switch (fault) {
      case ReactionWheel::RWFault::kNoFault:
        return "No Fault";
      case ReactionWheel::RWFault::kMoteusFault:
        return "Moteus Fault";
      case ReactionWheel::RWFault::kCommunicationError:
        return "Communication Error";
      case ReactionWheel::RWFault::kCANTimeout:
        return "Moteus CAN Timeout";
      default:
        return "Unknown";
    }
  }
}

// MAIN FLOW

void setup() {

  // Spin up the Serial interface for debug (10 second timeout)
  Serial.begin(115200);
  unsigned long timeout = millis();
  while (!Serial && (millis() - timeout < 10000)) {}

  // If there's a previous crash report we want to know about it
  if (CrashReport) {
    Serial.print(CrashReport);
    while(1); // Freeze here so you can read exactly what caused the crash!
  }

  if (Serial) {
    delay(1000);
    Serial.println("[main] Serial terminal connected");
  }

  // Set up initial parameters
  currentTelemetry.wheelStoredAngularMomentumKGM2S = wheelController.getAngularMomentum();
  currentTelemetry.thrustersPropellantRemainingKg = thrusterController.remainingPropellantKg();

  // Spin up CAN interface
  Serial.print("[main] Starting CAN interface...");
  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    Serial.printf("[main] [ERROR] CAN3 startup error 0x%X. Trying again in 1 second.\r\n", errorCode);
    delay(1000);
  }
  Serial.println("OK");

  // Spin up the OBC Link (SPI)
  Serial.print("[main] Initialising OBC SPI Link...");
  obcConnection.updateTelemetry(currentTelemetry); // Must set telemetry before calling begin() so that the first frame is valid
  obcConnection.begin();
  timeout = millis();
  while (!obcConnection.isConnected() && (millis() - timeout < 20000)) {
    Serial.print(".");
    delay(1000);
  }
  if (!obcConnection.isConnected()) {
    Serial.println("FAIL. Will keep trying in background. Check OBC is powered and connected.");
  } else {
    Serial.println("OK");
  }

  // Print out some debug if needed
  if (AOCS_SPI_DEBUG) {
    obcConnection.spiConnection().spiRegisterAudit();
  }

  // Spin up thruster control
  Serial.print("[main] Initialising Thruster Control...");
  thrusterController.begin();
  Serial.println("OK");
  
  // Spin up the reaction wheel controller
  // NOTE: This happens last in setup() intentionally because we need CAN messages to be sent
  // continuously after it's started or the motor controller will timeout.
  Serial.print("[main] Initialising Reaction Wheel Controller...");
  wheelController.begin();
  if (wheelController.status().rwMode != ReactionWheel::RWMode::kRunning) {
    Serial.println("FAIL. Fault: " + AirSat::reactionWheelFaultToString(wheelController.status().rwFault) + 
                   " | Moteus fault code: " + String(wheelController.status().motorControllerFaultCode));
  } else {
    Serial.println("OK");
  }
  
  Serial.println("[main] AOCS Control Startup Complete.");
  
}

void serviceSubSystems() {
  // Service the SPI connection to the OBC
  obcConnection.service();
  // Send commands / poll the reaction wheel controller
  wheelController.service();
}

void executeInstructions() {
  // Action any new instructions from the OBC
  if (obcConnection.hasNewCommand()) {
    CommandPayload workingCommand = obcConnection.takeLatestCommand();
    
    // Drive the reaction wheel with the requested torque
    wheelController.setTargetTorque(workingCommand.torque);
    currentTelemetry.aocsTargetTorqueNM = wheelController.getTargetTorque();

    thrusterController.setThrustN(workingCommand.thrust);
    currentTelemetry.thrustersPropellantRemainingKg = thrusterController.remainingPropellantKg();
  }
}

void refreshTelemetry() {

  // Thruster stats
  currentTelemetry.thrusters1ThrustN = thrusterController.getThrust0();
  currentTelemetry.thrusters2ThrustN = thrusterController.getThrust1();
  currentTelemetry.thrusters3ThrustN = thrusterController.getThrust2();
  currentTelemetry.thrusters4ThrustN = thrusterController.getThrust3();
  currentTelemetry.thrustersPropellantRemainingKg = thrusterController.remainingPropellantKg();

  // Reaction wheel stats
  currentTelemetry.wheelStoredAngularMomentumKGM2S = wheelController.getAngularMomentum();
  currentTelemetry.wheelMode = static_cast<uint8_t>(wheelController.status().rwMode);
  currentTelemetry.wheelFault = static_cast<uint8_t>(wheelController.status().rwFault);
  currentTelemetry.wheelTargetAccelerationRADSS = wheelController.getTargetAngularAcceleration();
  // OBC Link stats
  currentTelemetry.obcSyncDropCount = obcConnection.syncDropCount();
  currentTelemetry.obcCommandsRXCount = obcConnection.commandCount();
  currentTelemetry.obcNoOpCount = obcConnection.noOpCount();
  currentTelemetry.obcConnected = obcConnection.isConnected();
  currentTelemetry.obcRxErrorCount = obcConnection.rxErrorCount();
  currentTelemetry.obcTotalBytesReceived = obcConnection.totalBytesReceived();

  // Update OBC link with telemetry ready for next data transfer
  obcConnection.updateTelemetry(currentTelemetry);

}

void printDiagnostics() {
  if (Serial) {
    Serial.printf("[main] [OBC] Link state: %s | Commands RX: %u | No-Ops RX: %u | RX errors: %u | Sync drops: %u\r\n",
                  obcConnection.isConnected() ? "Connected" : "DISCONNECTED",
                  obcConnection.commandCount(),
                  obcConnection.noOpCount(),
                  obcConnection.rxErrorCount(), 
                  obcConnection.syncDropCount());
    Serial.printf("[main] [RW ] Stored Momentum: %.3f Kg.m²/s | Target Torque: %.3f Nm | Target Acceleration: %.3f rad/s² | Mode: %s | Fault: %s\r\n",
                  wheelController.getAngularMomentum(),
                  wheelController.getTargetTorque(),
                  wheelController.getTargetAngularAcceleration(),
                  AirSat::reactionWheelModeToString(wheelController.status().rwMode).c_str(),
                  AirSat::reactionWheelFaultToString(wheelController.status().rwFault).c_str());
    Serial.printf("[main] [RW ] Moteus mode: %u | Moteus fault code: %u\r\n",
                  wheelController.status().motorControllerMode,
                  wheelController.status().motorControllerFaultCode);
    Serial.printf("[main] [THR] Thrust1: %.3f N | Thrust2: %.3f N | Thrust3: %.3f N | Thrust4: %.3f N | Propellant remaining: %.2f kg\r\n",
                  thrusterController.getThrust0(),
                  thrusterController.getThrust1(),
                  thrusterController.getThrust2(),
                  thrusterController.getThrust3(),
                  thrusterController.remainingPropellantKg());

    if (THRUSTER_CONTROL_DEBUG) {
      Serial.printf("[main] [THR] Duty1: %.2f%% | Duty2: %.2f%% | Duty3: %.2f%% | Duty4: %.2f%%\r\n",
                    thrusterController.getDutyCycle0() * 100.0f,
                    thrusterController.getDutyCycle1() * 100.0f,
                    thrusterController.getDutyCycle2() * 100.0f,
                    thrusterController.getDutyCycle3() * 100.0f);
    }
    
    if (AOCS_SPI_DEBUG) {
      obcConnection.spiConnection().printSRRegisterDetail();
      obcConnection.spiConnection().printFSRRegisterDetail();
      obcConnection.spiConnection().printRSRRegisterDetail();
      SPIConnection::State spiState = obcConnection.spiConnection().state();
      Serial.printf("[main] [SPI] [DEBUG] SPI State: %s\r\n", 
                    (spiState == SPIConnection::State::Idle) ? "Idle" : 
                    (spiState == SPIConnection::State::Syncing) ? "Syncing" : 
                    (spiState == SPIConnection::State::Transceiving) ? "Transceiving" : "Unknown");

      SPIConnection::Stats spiStats = obcConnection.spiConnection().statsSnapshot();
      Serial.printf("[main] [SPI] [DEBUG] Byte RX ISR Calls: %u | Last Byte RX: %02X | Total Bytes RX: %u | Bytes RX in Last Interrupt: %u | FCFs Received: %u | REFs Received: %u | TX Errors: %u\r\n",
                    spiStats.byteRxISRCalls,
                    spiStats.lastByteReceived,
                    spiStats.totalBytesReceived,
                    spiStats.bytesReceivedInLastInterrupt,
                    spiStats.fcfsReceived,
                    spiStats.refsReceived,
                    spiStats.txErrorCount);

      Serial.printf("[main] [SPI] [DEBUG] Total Packets RX: %u | Discarded identical commands: %u | Bytes lost syncing: %u | Total checksum failures: %u | Completed frame drops: %u\r\n",
                    spiStats.totalPacketsReceived,
                    obcConnection.discardedCommandsCount(),
                    spiStats.bytesLostSyncing,
                    spiStats.checksumFailureCount,
                    spiStats.completedFrameDropCount);

      uint8_t rxPayloadSnapshot[sizeof(CommandPayload)] = {0};
      obcConnection.copyLastRxPayload(rxPayloadSnapshot, sizeof(rxPayloadSnapshot));
      uint8_t txFrameSnapshot[26] = {0}; 
      obcConnection.spiConnection().copyOutgoingTxFrame(txFrameSnapshot, sizeof(txFrameSnapshot));

      Serial.printf("[main] [SPI] [DEBUG] Last RX payload (%u bytes): ", static_cast<unsigned>(sizeof(rxPayloadSnapshot)));
      for (size_t i = 0; i < sizeof(rxPayloadSnapshot); ++i) {
        Serial.printf("%02X ", rxPayloadSnapshot[i]);
      }
      Serial.println();
      Serial.printf("[main] [SPI] [DEBUG] Scheduled TX Frame: ");
      for (size_t i = 0; i < sizeof(txFrameSnapshot); ++i) {
        Serial.printf("%02X ", txFrameSnapshot[i]);
      }
      Serial.println();
    }
  }
}

void loop() {
  // Process all updates across sub-systems
  serviceSubSystems();

  // Execute any new instructions from the OBC
  executeInstructions();

  // Local telemetry refresh
  static uint32_t telemetryTimer = 0;
  if (millis() - telemetryTimer >= 100) { 
    telemetryTimer = millis();
    refreshTelemetry();
  }

  // Diagnostics and printing
  static uint32_t diagnosticTimer = 0;
  if (millis() - diagnosticTimer >= 1000) {
    diagnosticTimer = millis();
    printDiagnostics();
  }
}
