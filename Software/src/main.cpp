#include <Arduino.h>
#include "OBCConnection.h"
#include "ReactionWheel.h"

OBCConnection obcConnection;
ReactionWheel wheelController(1);

TelemetryPayload currentTelemetry;
CommandPayload workingCommand;

void setup() {
  Serial.begin(115200);

  unsigned long timeout = millis();
  while (!Serial && (millis() - timeout < 10000)) {}

  if (Serial) {
    delay(1000);
    Serial.println("[main] Serial terminal connected");
    Serial.flush();
  }

  currentTelemetry.momentum = 0.0f;
  currentTelemetry.propellant = 1000; 

  // 1. Initialize the SPI Link FIRST while DMA tables are completely clear
  Serial.println("[main] Initialising OBC SPI Link...");
  Serial.flush();
  obcConnection.initialize();

  // 2. Safely create and initialize the Reaction Wheel module SECOND
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

