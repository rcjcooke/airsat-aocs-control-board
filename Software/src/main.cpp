#include <Arduino.h>
#include <ACAN_T4.h>

#include "AOCSControllerTelemetry.h"
#include "OBCConnection.h"
#include "ReactionWheel.h"

// The connection to the OBC (SPI)
OBCConnection obcConnection;

// CAN bus settings: 1 Mbps arbitration and 5 Mbps data rate for CAN-FD bus speed
ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x5);
// The reaction wheel controller
ReactionWheel wheelController(ACAN_T4::can3, canSettings, 1); 

// Current system telemetry
AOCSControllerTelemetry currentTelemetry;

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
    default:
      return "Unknown";
  }
}

void setup() {

  // Spin up the Serial interface for debug (10 second timeout)
  Serial.begin(115200);
  unsigned long timeout = millis();
  while (!Serial && (millis() - timeout < 10000)) {}
  if (Serial) {
    delay(1000);
    Serial.println("[main] Serial terminal connected");
  }

  // Set up initial parameters
  currentTelemetry.wheelStoredAngularMomentumKGM2S = 0.0f;
  currentTelemetry.thrustersPropellantRemainingM3 = 1000.0f;

  // Spin up the OBC Link (SPI)
  Serial.println("[main] Initialising OBC SPI Link...");
  Serial.flush();
  obcConnection.begin();

  // Spin up CAN interface
  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  if (errorCode != 0) {
    Serial.printf("Teensy CAN3 Hardware Failed: 0x%X\r\n", errorCode);
    // TODO: Handle errors better than this!!!
    while (1);
  }

  // Spin up the reaction wheel controller (CAN)
  Serial.println("[main] Initialising Reaction Wheel Controller...");
  Serial.flush();
  wheelController.begin();
  
  if (wheelController.status().rwMode != ReactionWheel::RWMode::kRunning) {
    Serial.println("[main] [WARN] Reaction Wheel offline at startup validation check.");
  } else {
    Serial.println("[main] AOCS Control Startup Complete.");
  }
}

void loop() {
  // Run sub-system loops
  wheelController.update();

  // Action any new instructions from the OBC
  if (obcConnection.hasNewCommand()) {
    CommandPayload workingCommand = obcConnection.takeLatestCommand();
    
    wheelController.setTargetTorque(workingCommand.torque);
    currentTelemetry.aocsTargetTorqueNM = wheelController.getTargetTorque();
  }

    // Local telemetry refresh
  static uint32_t telemetryTimer = 0;
  if (millis() - telemetryTimer >= 100) { 
    telemetryTimer = millis();

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

  // Diagnostics and printing
  static uint32_t diagnosticTimer = 0;
  if (millis() - diagnosticTimer >= 1000) {
    diagnosticTimer = millis();

    if (wheelController.status().rwMode != ReactionWheel::RWMode::kRunning) {
      Serial.printf("[main] [CRITICAL] Reaction Wheel Fault! Fault Code: 0x%02X | Mode: %d\r\n", 
                    static_cast<uint8_t>(wheelController.status().rwFault), static_cast<uint8_t>(wheelController.status().rwMode));
    }

    uint8_t rxPayloadSnapshot[sizeof(CommandPayload)] = {0};
    obcConnection.copyLastRxPayload(rxPayloadSnapshot, sizeof(rxPayloadSnapshot));

    Serial.printf("[main] [OBC] Link state: %s | Commands RX: %lu | No-Ops RX: %lu | RX errors: %u | Sync drops: %lu\r\n",
                  obcConnection.isConnected() ? "Connected" : "DISCONNECTED",
                  obcConnection.commandCount(),
                  obcConnection.noOpCount(),
                  obcConnection.rxErrorCount(), 
                  obcConnection.syncDropCount());
    Serial.printf("[main] [RW ] Stored Momentum: %f | Target Acceleration: %f | Mode: %s | Fault: %s\r\n",
                  wheelController.getAngularMomentum(),
                  wheelController.getTargetAngularAcceleration(),
                  reactionWheelModeToString(wheelController.status().rwMode),
                  reactionWheelFaultToString(wheelController.status().rwFault));
    
    // Serial.printf("[main] [OBC] Last RX payload (%u bytes): ", static_cast<unsigned>(sizeof(rxPayloadSnapshot)));
    // for (size_t i = 0; i < sizeof(rxPayloadSnapshot); ++i) {
    //   Serial.printf("%02X ", rxPayloadSnapshot[i]);
    // }
    // Serial.println();

  }
}
