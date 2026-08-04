#include <Arduino.h>

#include "SPIConnection.h"

namespace {
constexpr bool kSpiDebug = true;
constexpr uint8_t kSpiFrameSize = 26;
constexpr uint32_t kSerialBaudRate = 115200;
constexpr uint32_t kSerialWaitTimeoutMs = 10000;
constexpr uint32_t kDebugPrintPeriodMs = 4000;

SPIConnection g_spiConnection(kSpiDebug, kSpiFrameSize);
uint8_t g_bootTxFrame[SPIConnection::kMaxFrameSize] = {0};

void populateBootTxFrame() {
  for (uint8_t i = 0; i < g_spiConnection.frameSize(); ++i) {
    g_bootTxFrame[i] = i;
  }
}
}  // namespace

void setup() {
  Serial.begin(kSerialBaudRate);

  const unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < kSerialWaitTimeoutMs)) {
  }

  if (Serial) {
    delay(1000);
    Serial.println("[main] Serial terminal connected");
    Serial.flush();
  }

  populateBootTxFrame();
  g_spiConnection.setNextTxFrame(g_bootTxFrame, g_spiConnection.frameSize());
  g_spiConnection.begin();

  Serial.println("[main] Initialisation complete.");
  Serial.flush();
}

void loop() {
  static unsigned long lastPrintTime = 0;
  const unsigned long now = millis();

  if (now - lastPrintTime >= kDebugPrintPeriodMs) {
    lastPrintTime = now;
    g_spiConnection.printRuntimeDebug();
  }
}


