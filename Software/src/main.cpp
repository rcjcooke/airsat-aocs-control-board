#include <Arduino.h>
#include "OBCConnection.h"
#include "ReactionWheel.h"

OBCConnection obcLink;
ReactionWheel wheelController(1);

TelemetryPayload currentTelemetry;
CommandPayload workingCommand;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000); 

  currentTelemetry.momentum = 0.0f;
  currentTelemetry.propellant = 1000; 

  obcLink.begin();
  wheelController.begin();
  
  Serial.println("Flight Core Activated: Pure Command Router with Angular Momentum Telemetry.");
}

void loop() {
  wheelController.update();

  if (obcLink.isCommandAvailable()) {
    workingCommand = obcLink.getLatestCommand();
    wheelController.setTargetTorque(workingCommand.torque);
  }

  static uint32_t telemetryTimer = 0;
  if (millis() - telemetryTimer >= 100) { 
    telemetryTimer = millis();
    
    currentTelemetry.momentum = wheelController.getAngularMomentum(); 
    
    obcLink.updateTelemetry(currentTelemetry);
  }

  static uint32_t printTimer = 0;
  if (millis() - printTimer >= 1000) {
    printTimer = millis();
    
    // Diagnostics display comparing raw rotational velocity (Hz) alongside calculated momentum
    Serial.printf("[AOCS Control Log] CMD Torque: %.3f Nm | Wheel Speed: %.2f RPM | Stored Momentum: %.5f kg*m^2/s\n",
                  workingCommand.torque,
                  wheelController.getAngularVelocity() * 60.0f,
                  wheelController.getAngularMomentum());
  }
}
