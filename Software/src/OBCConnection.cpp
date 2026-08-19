#include "OBCConnection.h"

#include <string.h>

OBCConnection* OBCConnection::s_instance = nullptr;

OBCConnection::OBCConnection(bool spiDebug, uint8_t payloadSize)
    : m_spiConnection(spiDebug, payloadSize),
      m_commandMailbox{},
      m_telemetryPayload{},
      m_commandReadIndex(0),
      m_commandWriteIndex(0),
      m_newCommandReady(false),
      m_commandCount(0),
      m_noOpCount(0),
      m_malformedPacketCount(0),
      m_discardedCommandsCount(0),
      m_lastReceivedCommand{},
      m_hasLastReceivedCommand(false) {
      s_instance = this;
}

void OBCConnection::begin() {
  m_newCommandReady = false;
  m_commandReadIndex = 0;
  m_commandWriteIndex = 0;
  m_discardedCommandsCount = 0;
  m_hasLastReceivedCommand = false;
  memset(&m_commandMailbox[0], 0, sizeof(m_commandMailbox));
  memset(&m_lastReceivedCommand, 0, sizeof(m_lastReceivedCommand));
  memset(&m_telemetryPayload, 0, sizeof(m_telemetryPayload));

  m_spiConnection.setPayloadReadyHandler(onPayloadReceivedCallbackISR);
  queueTelemetryPayload();
  m_spiConnection.begin();
}

void OBCConnection::service() {
  m_spiConnection.service();
}

bool OBCConnection::hasNewCommand() const {
  return m_newCommandReady;
}

CommandPayload OBCConnection::takeLatestCommand() {
  const bool hadCommand = m_newCommandReady;
  m_newCommandReady = false;
  if (!hadCommand) {
    return CommandPayload{};
  }

  const uint8_t readIndex = m_commandReadIndex;
  return m_commandMailbox[readIndex];
}

void OBCConnection::updateTelemetry(const AOCSControllerTelemetry& telemetry) {
  // Create a new telemetry payload based on the provided AOCSControllerTelemetry data
  TelemetryPayload obcTelemetry;
  obcTelemetry.storedAngularMomentum = telemetry.wheelStoredAngularMomentumKGM2S;
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
  uint32_t count = m_malformedPacketCount + chkSumErrors;
  return count;
}

uint32_t OBCConnection::totalBytesReceived() const {
  return m_spiConnection.totalBytesReceived();
}

uint8_t OBCConnection::syncDropCount() const {
  return m_spiConnection.syncDropCount();
}

uint32_t OBCConnection::commandCount() const {
  return m_commandCount;
}

uint32_t OBCConnection::noOpCount() const {
  return m_noOpCount;
}

uint32_t OBCConnection::malformedPacketCount() const {
  return m_malformedPacketCount;
}

uint32_t OBCConnection::discardedCommandsCount() const {
  return m_discardedCommandsCount;
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
  OBCConnection* localInstance = s_instance;
  
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

  if (incomingPayload.flags == AOCSPacketConstants::kCommandPacket) {
    const bool hasLast = m_hasLastReceivedCommand;
    const bool isDuplicate =
        hasLast && (memcmp(&incomingPayload, &m_lastReceivedCommand, sizeof(CommandPayload)) == 0);

    m_lastReceivedCommand = incomingPayload;
    m_hasLastReceivedCommand = true;

    if (isDuplicate) {
      ++m_discardedCommandsCount;
      return;
    }

    const uint8_t writeIndex = m_commandWriteIndex;
    m_commandMailbox[writeIndex] = incomingPayload;
    m_commandReadIndex = writeIndex;
    m_commandWriteIndex = (writeIndex + 1U) % kCommandMailboxDepth;
    m_newCommandReady = true;
    ++m_commandCount;
  } else if (incomingPayload.flags == AOCSPacketConstants::kNoOpPacket) {
    ++m_noOpCount;
  } else {
    ++m_malformedPacketCount;
  }
}

void OBCConnection::queueTelemetryPayload() {
  m_spiConnection.setNextTxPayload(reinterpret_cast<const uint8_t*>(&m_telemetryPayload), sizeof(m_telemetryPayload));
}
