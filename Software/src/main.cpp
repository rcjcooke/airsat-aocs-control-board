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

  Serial.println("[main] Initialising OBC Link...");
  obcLink.begin();
  Serial.println("[main] Initialising Reaction Wheel Controller...");
  wheelController.begin();
  
  Serial.println("[main] AOCS Control Startup Complete.");
}

void loop() {
  wheelController.update();

  if (obcLink.isCommandAvailable()) {
    Serial.printf("[main] New command received from OBC: Torque=% .3f Nm, Thrusts= % .3f, % .3f, % .3f, % .3f N.", workingCommand.torque, workingCommand.thrust[0] , workingCommand.thrust[1], workingCommand.thrust[2], workingCommand.thrust[3]);
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
  if (millis() - printTimer >= 500) {
    printTimer = millis();
    
    // Diagnostics display comparing raw rotational velocity (Hz) alongside calculated momentum
    Serial.printf("[main] Target Torque: %.3f Nm | Wheel Speed: %.2f RPM | Stored Momentum: %.5f kg*m^2/s\r\n",
                  workingCommand.torque,
                  wheelController.getAngularVelocity() * 60.0f,
                  wheelController.getAngularMomentum());
  }
}
