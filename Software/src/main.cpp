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

  Serial.printf("[ALIGNMENT CHECK] CommandFrame Size: %d | TelemetryFrame Size: %d\r\n", 
              sizeof(CommandFrame), sizeof(TelemetryFrame));

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

  // Inside main.cpp loop() - replace your old 1Hz register check with this:
  static uint32_t multiPinAuditTimer = 0;
  if (millis() - multiPinAuditTimer >= 1000) {
    multiPinAuditTimer = millis();

    // 1. Snapshot all the raw IOMUX Multiplexer settings
    uint32_t muxCS   = IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00; // Pin 10
    uint32_t muxMOSI = IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02; // Pin 11
    uint32_t muxMISO = IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01; // Pin 12
    uint32_t muxSCK  = IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03; // Pin 13

    // 2. Snapshot all the raw Pad Control settings (pulls, speed, hysteresis)
    uint32_t padCS   = IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00; // Pin 10
    uint32_t padMOSI = IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02; // Pin 11
    uint32_t padMISO = IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01; // Pin 12
    uint32_t padSCK  = IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03; // Pin 13

    // 3. Snapshot the core GPIO7 direction bit states
    // Pin 10 = GPIO7 bit 0, Pin 11 = GPIO7 bit 2, Pin 12 = GPIO7 bit 1, Pin 13 = GPIO7 bit 3
    bool dirCS   = (GPIO7_GDIR >> 0) & 0x01;
    bool dirMOSI = (GPIO7_GDIR >> 2) & 0x01;
    bool dirMISO = (GPIO7_GDIR >> 1) & 0x01;
    bool dirSCK  = (GPIO7_GDIR >> 3) & 0x01;

    Serial.println("\n================= TEENSY SPI HARDWARE BUS SILICON AUDIT =================");
    
    // --- PIN 10: CHIP SELECT ---
    Serial.printf("PIN 10 [CS]   -> MUX (ALT): %u | GPIO Dir: %s | PAD Reg: 0x%04X ", 
                  muxCS & 0x07, dirCS ? "OUTPUT" : "INPUT", padCS);
    Serial.printf("(Schmitt: %s, Pulls: %s)\n", 
                  (padCS & (1 << 16)) ? "ON" : "OFF",
                  ((padCS >> 14) & 0x03) == 0 ? "NONE" : ((padCS >> 14) & 0x03) == 1 ? "100k-DOWN" : "100k-UP");

    // --- PIN 11: MOSI ---
    Serial.printf("PIN 11 [MOSI] -> MUX (ALT): %u | GPIO Dir: %s | PAD Reg: 0x%04X ", 
                  muxMOSI & 0x07, dirMOSI ? "OUTPUT" : "INPUT", padMOSI);
    Serial.printf("(Schmitt: %s, Pulls: %s)\n", 
                  (padMOSI & (1 << 16)) ? "ON" : "OFF",
                  ((padMOSI >> 14) & 0x03) == 0 ? "NONE" : ((padMOSI >> 14) & 0x03) == 1 ? "100k-DOWN" : "100k-UP");

    // --- PIN 12: MISO ---
    Serial.printf("PIN 12 [MISO] -> MUX (ALT): %u | GPIO Dir: %s | PAD Reg: 0x%04X ", 
                  muxMISO & 0x07, dirMISO ? "OUTPUT" : "INPUT", padMISO);
    Serial.printf("(Schmitt: %s, Pulls: %s)\n", 
                  (padMISO & (1 << 16)) ? "ON" : "OFF",
                  ((padMISO >> 14) & 0x03) == 0 ? "NONE" : ((padMISO >> 14) & 0x03) == 1 ? "100k-DOWN" : "100k-UP");

    // --- PIN 13: SCK ---
    Serial.printf("PIN 13 [SCK]  -> MUX (ALT): %u | GPIO Dir: %s | PAD Reg: 0x%04X ", 
                  muxSCK & 0x07, dirSCK ? "OUTPUT" : "INPUT", padSCK);
    Serial.printf("(Schmitt: %s, Pulls: %s)\n", 
                  (padSCK & (1 << 16)) ? "ON" : "OFF",
                  ((padSCK >> 14) & 0x03) == 0 ? "NONE" : ((padSCK >> 14) & 0x03) == 1 ? "100k-DOWN" : "100k-UP");

    // --- CHIP-LEVEL CONTROL OVERRIDES ---
    Serial.printf("Daisy Chains -> SDO_SELECT: 0x%X | SDI_SELECT: 0x%X\n", 
                  IOMUXC_LPSPI4_SDO_SELECT_INPUT, IOMUXC_LPSPI4_SDI_SELECT_INPUT); // Note: Core macro may be DI or SDI
    Serial.printf("LPSPI4 Status -> CR: 0x%08X | IER: 0x%08X | SR: 0x%08X\n", 
                  LPSPI4_CR, LPSPI4_IER, LPSPI4_SR);
                  
    Serial.println("=========================================================================");
    Serial.flush();
  }


  // ------------------------------------------------------------------------
  // UPDATED: DIAGNOSTIC HEX STREAM MONITOR (1Hz)
  // ------------------------------------------------------------------------
  static uint32_t diagnosticTimer = 0;
  if (millis() - diagnosticTimer >= 1000) {
    diagnosticTimer = millis();
    
    uint8_t rxSnapshot[26];
    getOBCRawRxBufferSnapshot(rxSnapshot, 26);

    Serial.printf("[TEENSY SPI LOG] Total Bytes: %u | Framing Errors: %u\r\n", 
                  getOBCTotalBytesReceived(), 
                  getOBCRxErrorCount());
                  
    Serial.print("[TEENSY SPI LOG] Raw Data Hex Dump: ");
    for (int i = 0; i < 26; ++i) {
      Serial.printf("%02X ", rxSnapshot[i]);
    }
    Serial.println("\r\n------------------------------------------------------------------------");
    Serial.flush();
  }
  // ------------------------------------------------------------------------
}

