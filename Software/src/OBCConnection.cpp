#include "OBCConnection.h"

#include <string.h>

OBCConnection* OBCConnection::s_instance = nullptr;

OBCConnection::OBCConnection(bool spiDebug, uint8_t payloadSize)
    : m_spiConnection(spiDebug, payloadSize),
      m_newCommandReady(false),
      m_verifiedCommand{},
      m_telemetryPayload{},
      m_commandCount(0),
      m_noOpCount(0) {
  s_instance = this;
}

void OBCConnection::initialize() {
  noInterrupts();
  m_newCommandReady = false;
  memset(&m_verifiedCommand, 0, sizeof(m_verifiedCommand));
  memset(&m_telemetryPayload, 0, sizeof(m_telemetryPayload));
  interrupts();

  m_spiConnection.setPayloadReadyHandler(onPayloadReceivedCallback);
  queueTelemetryPayload();
  m_spiConnection.begin();
}

bool OBCConnection::hasNewCommand() const {
  return m_newCommandReady;
}

CommandPayload OBCConnection::takeLatestCommand() {
  CommandPayload command;
  noInterrupts();
  command = m_verifiedCommand;
  m_newCommandReady = false;
  interrupts();
  return command;
}

void OBCConnection::updateTelemetry(const TelemetryPayload& telemetry) {
  noInterrupts();
  m_telemetryPayload = telemetry;
  m_telemetryPayload.error_count = rxErrorCount();
  interrupts();

  queueTelemetryPayload();
}

uint16_t OBCConnection::rxErrorCount() const {
  const SPIConnection::Stats stats = m_spiConnection.statsSnapshot();
  return static_cast<uint16_t>(stats.checksumFailureCount + stats.partialFrameErrorCount);
}

uint32_t OBCConnection::totalBytesReceived() const {
  return m_spiConnection.statsSnapshot().totalBytesReceived;
}

uint32_t OBCConnection::totalInterruptsReceived() const {
  return m_spiConnection.statsSnapshot().interruptCalls;
}

uint32_t OBCConnection::syncDropCount() const {
  return m_spiConnection.statsSnapshot().syncDropCount;
}

uint32_t OBCConnection::commandCount() const {
  return m_commandCount;
}

uint32_t OBCConnection::noOpCount() const {
  return m_noOpCount;
}

void OBCConnection::copyLastRxPayload(uint8_t* destinationBuffer, size_t maxBytes) const {
  if (destinationBuffer == nullptr || maxBytes == 0) {
    return;
  }

  m_spiConnection.copyLastRxPayload(destinationBuffer, maxBytes);
}

void OBCConnection::onPayloadReceivedCallback(const uint8_t* payload, uint8_t payloadSize) {
  if (s_instance != nullptr) {
    s_instance->onPayloadReceived(payload, payloadSize);
  }
}

void OBCConnection::onPayloadReceived(const uint8_t* payload, uint8_t payloadSize) {
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
  }
}

void OBCConnection::queueTelemetryPayload() {
  m_spiConnection.setNextTxPayload(reinterpret_cast<const uint8_t*>(&m_telemetryPayload), sizeof(m_telemetryPayload));
}
