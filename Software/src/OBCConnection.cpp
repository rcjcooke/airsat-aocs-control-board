#include "OBCConnection.h"

#include <string.h>

OBCConnection* OBCConnection::s_instance = nullptr;

OBCConnection::OBCConnection(bool spiDebug, uint8_t frameSize)
    : m_spiConnection(spiDebug, frameSize),
      m_newCommandReady(false),
      m_localErrorCount(0),
      m_verifiedCommand{},
      m_outgoingFrame{} {
  s_instance = this;
}

void OBCConnection::initialize() {
  noInterrupts();
  m_newCommandReady = false;
  m_localErrorCount = 0;
  memset(&m_verifiedCommand, 0, sizeof(m_verifiedCommand));
  memset(&m_outgoingFrame, 0, sizeof(m_outgoingFrame));
  refreshTelemetryTxFrame();
  interrupts();

  m_spiConnection.setFrameReadyHandler(onFrameReceivedCallback);
  m_spiConnection.setNextTxFrame(reinterpret_cast<const uint8_t*>(&m_outgoingFrame), sizeof(TelemetryFrame));
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
  m_outgoingFrame.payload = telemetry;
  refreshTelemetryTxFrame();
  interrupts();

  m_spiConnection.setNextTxFrame(reinterpret_cast<const uint8_t*>(&m_outgoingFrame), sizeof(TelemetryFrame));
}

uint16_t OBCConnection::rxErrorCount() const {
  return m_localErrorCount;
}

uint32_t OBCConnection::totalBytesReceived() const {
  return m_spiConnection.statsSnapshot().totalBytesReceived;
}

uint32_t OBCConnection::totalInterruptsReceived() const {
  return m_spiConnection.statsSnapshot().interruptCalls;
}

uint32_t OBCConnection::csFallingEdges() const {
  return 0;
}

uint32_t OBCConnection::csLineState() const {
  return digitalReadFast(10);
}

void OBCConnection::copyLastRxFrame(uint8_t* destinationBuffer, size_t maxBytes) const {
  if (destinationBuffer == nullptr || maxBytes == 0) {
    return;
  }

  m_spiConnection.copyLastRxFrame(destinationBuffer, maxBytes);
}

void OBCConnection::onFrameReceivedCallback(const uint8_t* frame, uint8_t frameSize) {
  if (s_instance != nullptr) {
    s_instance->onFrameReceived(frame, frameSize);
  }
}

void OBCConnection::onFrameReceived(const uint8_t* frame, uint8_t frameSize) {
  if (frame == nullptr || frameSize != sizeof(CommandFrame)) {
    ++m_localErrorCount;
    return;
  }

  CommandFrame incomingFrame;
  memcpy(&incomingFrame, frame, sizeof(incomingFrame));

  if (incomingFrame.sync[0] != kSyncByte0 || incomingFrame.sync[1] != kSyncByte1) {
    ++m_localErrorCount;
    return;
  }

  const uint16_t calculatedChecksum =
      calculateFletcher16(reinterpret_cast<const uint8_t*>(&incomingFrame), sizeof(CommandFrame) - sizeof(uint16_t));

  if (calculatedChecksum != incomingFrame.checksum) {
    ++m_localErrorCount;
    return;
  }

  if (incomingFrame.payload.flags == 0x11) {
    m_verifiedCommand = incomingFrame.payload;
    m_newCommandReady = true;
  }
}

void OBCConnection::refreshTelemetryTxFrame() {
  m_outgoingFrame.sync[0] = kSyncByte0;
  m_outgoingFrame.sync[1] = kSyncByte1;
  m_outgoingFrame.payload.error_count = m_localErrorCount;
  m_outgoingFrame.checksum =
      calculateFletcher16(reinterpret_cast<const uint8_t*>(&m_outgoingFrame), sizeof(TelemetryFrame) - sizeof(uint16_t));
}

uint16_t OBCConnection::calculateFletcher16(const uint8_t* data, size_t count) {
  uint16_t sum1 = 0;
  uint16_t sum2 = 0;

  for (size_t i = 0; i < count; ++i) {
    sum1 = (sum1 + data[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }

  return static_cast<uint16_t>((sum2 << 8) | sum1);
}
