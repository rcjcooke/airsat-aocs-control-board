#include <Arduino.h>
#include "OBCConnection.h"
#include "ReactionWheel.h"

ReactionWheel* wheelController = nullptr;

TelemetryPayload currentTelemetry;
CommandPayload workingCommand;

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  unsigned long timeout = millis();
  while (!Serial && (millis() - timeout < 10000)) {
    digitalWrite(LED_BUILTIN, (millis() / 250) % 2); 
  }
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LED_BUILTIN, INPUT);

  pinMode(10, INPUT_PULLUP); // Secure CS Pin 10
  delay(1000);

  if (Serial) {
    Serial.println("[main] Serial terminal connected");
    Serial.flush();
  }

  currentTelemetry.momentum = 0.0f;
  currentTelemetry.propellant = 1000; 

  // 1. Initialize the SPI Link FIRST while DMA tables are completely clear
  Serial.println("[main] Initialising OBC SPI Link...");
  Serial.flush();
  initOBCConnection(); 

  // 2. Safely create and initialize the Reaction Wheel module SECOND
  Serial.println("[main] Initialising Reaction Wheel Controller...");
  Serial.flush();
  wheelController = new ReactionWheel(1); // Allocated safely post-SPI boot
  wheelController->begin();
  
  Serial.println("[main] AOCS Control Startup Complete.");
  Serial.flush();
}

void loop() {
  // Service the pointer object safely
  if (wheelController) {
    wheelController->update();
  }

  if (isOBCCommandAvailable()) {
    workingCommand = getLatestOBCCommand();
    if (wheelController) {
      wheelController->setTargetTorque(workingCommand.torque);
    }
  }

  static uint32_t telemetryTimer = 0;
  if (millis() - telemetryTimer >= 100) { 
    telemetryTimer = millis();
    
    if (wheelController) {
      currentTelemetry.momentum = wheelController->getAngularMomentum(); 
    }
    
    updateOBCTelemetry(currentTelemetry);
  }
}
