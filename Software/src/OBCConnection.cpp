#include "OBCConnection.h"

#include <string.h>

volatile OBCConnection* OBCConnection::s_instance = nullptr;

OBCConnection::OBCConnection(bool spiDebug, uint8_t payloadSize)
    : m_spiConnection(spiDebug, payloadSize),
  m_commandMailbox{},
      m_telemetryPayload{},
  m_commandReadIndex(0),
  m_commandWriteIndex(0),
      m_newCommandReady(false),
      m_commandCount(0),
      m_noOpCount(0),
      m_malformedFrame(0) {
  s_instance = this;
}

void OBCConnection::begin() {
  m_newCommandReady.store(false, std::memory_order_relaxed);
  m_commandReadIndex.store(0, std::memory_order_relaxed);
  m_commandWriteIndex.store(0, std::memory_order_relaxed);
  memset(&m_commandMailbox[0], 0, sizeof(m_commandMailbox));
  memset(&m_telemetryPayload, 0, sizeof(m_telemetryPayload));

  m_spiConnection.setPayloadReadyHandler(onPayloadReceivedCallbackISR);
  queueTelemetryPayload();
  m_spiConnection.begin();
}

void OBCConnection::activateSPI() {
  m_spiConnection.activate();
}

bool OBCConnection::hasNewCommand() const {
  return m_newCommandReady.load(std::memory_order_acquire);
}

CommandPayload OBCConnection::takeLatestCommand() {
  const bool hadCommand = m_newCommandReady.exchange(false, std::memory_order_acq_rel);
  if (!hadCommand) {
    return CommandPayload{};
  }

  const uint8_t readIndex = m_commandReadIndex.load(std::memory_order_acquire);
  return m_commandMailbox[readIndex];
}

void OBCConnection::updateTelemetry(const AOCSControllerTelemetry& telemetry) {
  // Create a new telemetry payload based on the provided AOCSControllerTelemetry data
  TelemetryPayload obcTelemetry;
  obcTelemetry.momentum = telemetry.wheelStoredAngularMomentumKGM2S;
  obcTelemetry.propellant = static_cast<uint16_t>(telemetry.thrustersPropellantRemainingKg);
  obcTelemetry.error_count = rxErrorCount();

  // Switch it over
  m_telemetryPayload = obcTelemetry;

  queueTelemetryPayload();
}

bool OBCConnection::isConnected() const {
  return m_spiConnection.state() == SPIConnection::State::Transceiving;
}

uint32_t OBCConnection::rxErrorCount() const {
  uint32_t chkSumErrors = m_spiConnection.checksumFailureCount();
  uint32_t count = m_malformedFrame.load(std::memory_order_relaxed) + chkSumErrors;
  return count;
}

uint32_t OBCConnection::totalBytesReceived() const {
  return m_spiConnection.totalBytesReceived();
}

uint8_t OBCConnection::syncDropCount() const {
  return m_spiConnection.syncDropCount();
}

uint8_t OBCConnection::commandCount() const {
  return static_cast<uint8_t>(m_commandCount.load(std::memory_order_relaxed));
}

uint8_t OBCConnection::noOpCount() const {
  return static_cast<uint8_t>(m_noOpCount.load(std::memory_order_relaxed));
}

uint8_t OBCConnection::malformedFrameCount() const {
  return static_cast<uint8_t>(m_malformedFrame.load(std::memory_order_relaxed));
}

void OBCConnection::copyLastRxPayload(uint8_t* destinationBuffer, size_t maxBytes) const {
  if (destinationBuffer == nullptr || maxBytes == 0) {
    return;
  }

  m_spiConnection.copyLastRxPayload(destinationBuffer, maxBytes);
}

SPIConnection& OBCConnection::spiConnection() {
  return m_spiConnection;
}

void OBCConnection::onPayloadReceivedCallbackISR(const uint8_t* payload, uint8_t payloadSize) {
  OBCConnection* localInstance = (OBCConnection*)s_instance;
  
  if (localInstance != nullptr) {
    localInstance->onPayloadReceivedISR(payload, payloadSize);
  }
}

void OBCConnection::onPayloadReceivedISR(const uint8_t* payload, uint8_t payloadSize) {
  if (payload == nullptr || payloadSize != sizeof(CommandPayload)) {
    return;
  }

  CommandPayload incomingPayload;
  memcpy(&incomingPayload, payload, sizeof(incomingPayload));

  if (incomingPayload.flags == kCommandFrame) {
    const uint8_t writeIndex = m_commandWriteIndex.load(std::memory_order_relaxed);
    m_commandMailbox[writeIndex] = incomingPayload;
    m_commandReadIndex.store(writeIndex, std::memory_order_release);
    m_commandWriteIndex.store((writeIndex + 1U) % kCommandMailboxDepth, std::memory_order_relaxed);
    m_newCommandReady.store(true, std::memory_order_release);
    m_commandCount.fetch_add(1, std::memory_order_relaxed);
  } else if (incomingPayload.flags == kNoOpFrame) {
    m_noOpCount.fetch_add(1, std::memory_order_relaxed);
  } else {
    m_malformedFrame.fetch_add(1, std::memory_order_relaxed);
  }
}

void OBCConnection::queueTelemetryPayload() {
  m_spiConnection.setNextTxPayload(reinterpret_cast<const uint8_t*>(&m_telemetryPayload), sizeof(m_telemetryPayload));
}
