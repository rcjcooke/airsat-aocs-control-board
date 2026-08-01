#include <Arduino.h>
#include "OBCConnection.h"
#include "ReactionWheel.h"

OBCConnection obcLink;
ReactionWheel wheelController(1);

TelemetryPayload currentTelemetry;
CommandPayload workingCommand;

void setup() {
  Serial.begin(115200);

  // Blink the built-in LED while waiting for Serial connection for up to 10 seconds
  pinMode(LED_BUILTIN, OUTPUT);
  unsigned long timeout = millis();
  while (!Serial && (millis() - timeout < 10000)) {
    digitalWrite(LED_BUILTIN, (millis() / 250) % 2); 
  }
  if (Serial) {
    delay(200); 
    Serial.println("[main] Serial terminal connected");
    Serial.flush(); // Force the USB buffer to transmit immediately
  }

  digitalWrite(LED_BUILTIN, HIGH); // This will only hold true until we set up SPI as the same pin is used for SPI SCK.

  // Force SPI CS high so it only activates when the Pi explicitly drives it low
  pinMode(10, INPUT_PULLUP); 

  currentTelemetry.momentum = 0.0f;
  currentTelemetry.propellant = 1000; 

  Serial.println("[main] Initialising OBC Link...");
  Serial.flush(); // Force the USB buffer to transmit immediately
  obcLink.begin();
  Serial.println("[main] Initialising Reaction Wheel Controller...");
  Serial.flush(); // Force the USB buffer to transmit immediately
  wheelController.begin();
  
  Serial.println("[main] AOCS Control Startup Complete.");
  Serial.flush(); // Force the USB buffer to transmit immediately
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
