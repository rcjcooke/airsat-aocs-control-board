#include <Arduino.h>
#include <ACAN_T4.h>

#include "OBCConnection.h"
#include "ReactionWheel.h"

OBCConnection obcConnection;

// Define 1 Mbps arbitration and 5 Mbps data rate for CAN-FD bus speed
ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x5);

ReactionWheel wheelController(ACAN_T4::can3, canSettings, 1); 
TelemetryPayload currentTelemetry;
CommandPayload workingCommand;

void setup() {

  // Spin up the Serial interface for debug (10 second timeout)
  Serial.begin(115200);
  unsigned long timeout = millis();
  while (!Serial && (millis() - timeout < 10000)) {}
  if (Serial) {
    delay(1000);
    Serial.println("[main] Serial terminal connected");
    Serial.flush();
  }

  // Set up initial parameters
  currentTelemetry.momentum = 0.0f;
  currentTelemetry.propellant = 1000; 

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
  
  Serial.println("[main] AOCS Control Startup Complete.");
  Serial.flush();
}

void loop() {
  wheelController.update();

  if (obcConnection.hasNewCommand()) {
    workingCommand = obcConnection.takeLatestCommand();
    wheelController.setTargetTorque(workingCommand.torque);
  }

  static uint32_t telemetryTimer = 0;
  if (millis() - telemetryTimer >= 100) { 
    telemetryTimer = millis();
    currentTelemetry.momentum = wheelController.getAngularMomentum();
    obcConnection.updateTelemetry(currentTelemetry);
  }

  static uint32_t diagnosticTimer = 0;
  if (millis() - diagnosticTimer >= 1000) {
    diagnosticTimer = millis();

    uint8_t rxPayloadSnapshot[sizeof(CommandPayload)] = {0};
    obcConnection.copyLastRxPayload(rxPayloadSnapshot, sizeof(rxPayloadSnapshot));

    Serial.printf("[main] [OBC] RX bytes: %lu | Commands RX: %lu | No-Ops RX: %lu | RX errors: %u | Sync drops: %lu\r\n",
                  obcConnection.totalBytesReceived(),
                  obcConnection.commandCount(),
                  obcConnection.noOpCount(),
                  obcConnection.rxErrorCount(),
                  obcConnection.syncDropCount());

    Serial.printf("[main] [OBC] Last RX payload (%u bytes): ", static_cast<unsigned>(sizeof(rxPayloadSnapshot)));
    for (size_t i = 0; i < sizeof(rxPayloadSnapshot); ++i) {
      Serial.printf("%02X ", rxPayloadSnapshot[i]);
    }
    Serial.println();
    Serial.flush();
  }
}

