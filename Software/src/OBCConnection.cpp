#include "OBCConnection.h"

#include <string.h>

OBCConnection* OBCConnection::s_instance = nullptr;

OBCConnection::OBCConnection(bool spiDebug, uint8_t payloadSize)
    : m_spiConnection(spiDebug, payloadSize),
      m_verifiedCommand{},
      m_telemetryPayload{},
      m_newCommandReady(false),
      m_commandCount(0),
      m_noOpCount(0),
      m_malformedFrame(0) {
  s_instance = this;
}

void OBCConnection::begin() {
  noInterrupts();
  m_newCommandReady = false;
  memset(&m_verifiedCommand, 0, sizeof(m_verifiedCommand));
  memset(&m_telemetryPayload, 0, sizeof(m_telemetryPayload));
  interrupts();

  m_spiConnection.setPayloadReadyHandler(onPayloadReceivedCallbackISR);
  queueTelemetryPayload();
  m_spiConnection.begin();
}

bool OBCConnection::hasNewCommand() const {
  return m_newCommandReady;
}

CommandPayload OBCConnection::takeLatestCommand() {
  noInterrupts(); // Stop SPI interrupts from breaking the copy
  CommandPayload localCopy = m_verifiedCommand;
  m_newCommandReady = false;
  interrupts();   // Safe to resume interrupts
  
  return localCopy;
}

void OBCConnection::updateTelemetry(const AOCSControllerTelemetry& telemetry) {
  // Create a new telemetry payload based on the provided AOCSControllerTelemetry data
  TelemetryPayload obcTelemetry;
  obcTelemetry.momentum = telemetry.wheelStoredAngularMomentumKGM2S;
  obcTelemetry.propellant = static_cast<uint16_t>(telemetry.thrustersPropellantRemainingM3 * 1000.0f); // Convert to milliliters
  obcTelemetry.error_count = rxErrorCount();

  // Switch it over
  noInterrupts();
  m_telemetryPayload = obcTelemetry;
  interrupts();

  queueTelemetryPayload();
}

bool OBCConnection::isConnected() const {
  return m_spiConnection.state() == SPIConnection::State::Transceiving;
}

uint8_t OBCConnection::rxErrorCount() const {
  return m_malformedFrame + m_spiConnection.checksumFailureCount();
}

uint32_t OBCConnection::totalBytesReceived() const {
  return m_spiConnection.totalBytesReceived();
}

uint8_t OBCConnection::syncDropCount() const {
  return m_spiConnection.syncDropCount();
}

uint8_t OBCConnection::commandCount() const {
  noInterrupts();
  uint8_t count = m_commandCount;
  interrupts();
  return count;
}

uint8_t OBCConnection::noOpCount() const {
  noInterrupts();
  uint8_t count = m_noOpCount;
  interrupts();
  return count;
}

void OBCConnection::copyLastRxPayload(uint8_t* destinationBuffer, size_t maxBytes) const {
  if (destinationBuffer == nullptr || maxBytes == 0) {
    return;
  }

  m_spiConnection.copyLastRxPayload(destinationBuffer, maxBytes);
}

void OBCConnection::onPayloadReceivedCallbackISR(const uint8_t* payload, uint8_t payloadSize) {
  if (s_instance != nullptr) {
    s_instance->onPayloadReceivedISR(payload, payloadSize);
  }
}

void OBCConnection::onPayloadReceivedISR(const uint8_t* payload, uint8_t payloadSize) {
  if (payload == nullptr || payloadSize != sizeof(CommandPayload)) {
    return;
  }

  CommandPayload incomingPayload;
  memcpy(&incomingPayload, payload, sizeof(incomingPayload));

  if (incomingPayload.flags == 0x11) {
    m_verifiedCommand = incomingPayload;
    m_newCommandReady = true;
    m_commandCount++;
  } else if (incomingPayload.flags == 0x22) {
    m_noOpCount++;
  } else {
    m_malformedFrame++;
  }
}

void OBCConnection::queueTelemetryPayload() {
  m_spiConnection.setNextTxPayload(reinterpret_cast<const uint8_t*>(&m_telemetryPayload), sizeof(m_telemetryPayload));
}
