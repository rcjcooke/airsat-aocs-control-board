#include "SPIConnection.h"

#include <SPISlave_T4.h>

#include <string.h>

namespace {
SPISlave_T4<&SPI, SPI_8_BITS> g_spi;

uint8_t clampPayloadSize(uint8_t requestedPayloadSize) {
  if (requestedPayloadSize == 0) {
    return 1;
  }
  if (requestedPayloadSize > SPIConnection::kMaxPayloadSize) {
    return SPIConnection::kMaxPayloadSize;
  }
  return requestedPayloadSize;
}
}  // namespace

SPIConnection* SPIConnection::s_instance = nullptr;

SPIConnection::SPIConnection(bool debugEnabled, uint8_t payloadSize)
    : m_isActive(false),
      m_debugEnabled(debugEnabled),
      m_payloadSize(clampPayloadSize(payloadSize)),
      m_frameSize(static_cast<uint8_t>(m_payloadSize + kFrameOverhead)),
      m_state(State::Idle),
      m_spiInputBuffer{0},
      m_spiOutputBuffer{0},
      m_spiBufferIndex(0),
      m_frameSynced(false),
      m_lastRxPayload{0},
      m_nextTxFrame{0},
      m_interruptCalls(0),
      m_lastByteReceived(0),
      m_totalBytesReceived(0),
      m_totalFramesReceived(0),
      m_checksumFailureCount(0),
      m_syncDropCount(0),
      m_bytesLostSyncing(0),
      m_bytesReceivedInLastInterrupt(0),
      m_uncheckedInterruptDebugData(false),
      m_fcfsReceived(0),
      m_txErrorCount(0),
      m_payloadReadyHandler(nullptr) {
  buildTxFrameFromPayload(nullptr, 0);
}

void SPIConnection::begin() {
  s_instance = this;

  m_state = State::Idle;
  m_spiBufferIndex = 0;
  m_frameSynced = false;
  m_uncheckedInterruptDebugData = false;
  m_isActive = false;

  memcpy((void*)m_spiOutputBuffer, (const void*)m_nextTxFrame, m_frameSize);

  g_spi.setIERTriggerMode(true, false);
  g_spi.begin();
  g_spi.onReceive(handleMessageISR);

  if (m_debugEnabled) {
    Serial.println("[SPI] Transport initialised");
    Serial.flush();
  }
}

void SPIConnection::activate() {
  m_isActive = true;
  if (m_debugEnabled) {
    Serial.println("[SPI] Message processing activated");
    Serial.flush();
  }
}

void SPIConnection::setPayloadReadyHandler(PayloadReadyHandler handler) {
  noInterrupts();
  m_payloadReadyHandler = handler;
  interrupts();
}

void SPIConnection::setNextTxPayload(const uint8_t* payload, size_t payloadSize) {
  noInterrupts();
  buildTxFrameFromPayload(payload, payloadSize);
  interrupts();
}

void SPIConnection::copyLastRxPayload(uint8_t* destination, size_t maxBytes) const {
  if (destination == nullptr || maxBytes == 0) {
    return;
  }

  const size_t boundedSize = (maxBytes < m_payloadSize) ? maxBytes : m_payloadSize;

  noInterrupts();
  memcpy(destination, (const void*)m_lastRxPayload, boundedSize);
  interrupts();
}

uint8_t SPIConnection::payloadSize() const {
  return m_payloadSize;
}

uint8_t SPIConnection::frameSize() const {
  return m_frameSize;
}

uint32_t SPIConnection::totalBytesReceived() const {
  uint32_t totalBytes = 0;
  noInterrupts();
  totalBytes = m_totalBytesReceived;
  interrupts();
  return totalBytes;
}

uint8_t SPIConnection::syncDropCount() const {
  // 8-bit so no need for protection with interrupts
  return m_syncDropCount;
}

uint8_t SPIConnection::checksumFailureCount() const {
  // 8-bit so no need for protection with interrupts
  return m_checksumFailureCount;
}

SPIConnection::Stats SPIConnection::statsSnapshot() const {
  noInterrupts();
  const Stats snapshot = {
      m_interruptCalls,
      m_lastByteReceived,
      m_totalBytesReceived,
      m_totalFramesReceived,
      m_bytesLostSyncing,
      m_bytesReceivedInLastInterrupt,
      m_fcfsReceived,
      m_txErrorCount};
  interrupts();
  return snapshot;
}

SPIConnection::State SPIConnection::state() const {
  // 8-bit so no need for protection with interrupts
  return m_state;
}

uint16_t SPIConnection::calculateFletcher16(const uint8_t* data, size_t count) {
  uint16_t sum1 = 0;
  uint16_t sum2 = 0;

  for (size_t i = 0; i < count; ++i) {
    sum1 = static_cast<uint16_t>((sum1 + data[i]) % 255);
    sum2 = static_cast<uint16_t>((sum2 + sum1) % 255);
  }

  return static_cast<uint16_t>((sum2 << 8) | sum1);
}

void SPIConnection::buildTxFrameFromPayload(const uint8_t* payload, size_t payloadSize) {
  const size_t boundedSize = (payloadSize < m_payloadSize) ? payloadSize : m_payloadSize;

  memset(m_nextTxFrame, 0, m_frameSize);
  m_nextTxFrame[0] = kSyncByte0;
  m_nextTxFrame[1] = kSyncByte1;

  if (payload != nullptr && boundedSize > 0) {
    memcpy(&m_nextTxFrame[kSyncSize], payload, boundedSize);
  }

  const uint16_t checksum = calculateFletcher16(m_nextTxFrame, m_frameSize - kChecksumSize);
  m_nextTxFrame[m_frameSize - 2] = static_cast<uint8_t>(checksum & 0xFF);
  m_nextTxFrame[m_frameSize - 1] = static_cast<uint8_t>((checksum >> 8) & 0xFF);
}

void SPIConnection::validateAndDispatchFrame() {
  const uint16_t calculatedChecksum =
      calculateFletcher16(m_spiInputBuffer, m_frameSize - kChecksumSize);
  const uint16_t receivedChecksum =
      static_cast<uint16_t>(m_spiInputBuffer[m_frameSize - 2]) |
      (static_cast<uint16_t>(m_spiInputBuffer[m_frameSize - 1]) << 8);

  if (calculatedChecksum != receivedChecksum) {
    ++m_checksumFailureCount;
    return;
  }

  memcpy((void*)m_lastRxPayload, (const void*)&m_spiInputBuffer[kSyncSize], m_payloadSize);

  if (m_isActive) {
    // SAFE COPY TO PREVENT NULLPTR FAULTS:
    PayloadReadyHandler handlerCopy = (PayloadReadyHandler) m_payloadReadyHandler;
    if (handlerCopy != nullptr) {
      handlerCopy((const uint8_t*)m_lastRxPayload, m_payloadSize);
    }
  }

  ++m_totalFramesReceived;
}

void SPIConnection::handleMessageISR() {
  if (s_instance != nullptr) {
    s_instance->handleMessage();
  }
}

void SPIConnection::handleMessage() {
  ++m_interruptCalls;
  if (!m_uncheckedInterruptDebugData) {
    m_bytesReceivedInLastInterrupt = 0;
  }

  while (g_spi.isDataAvailable()) {
    const uint8_t rxData = static_cast<uint8_t>(g_spi.popr());
    m_lastByteReceived = rxData;
    ++m_totalBytesReceived;

    if (!m_uncheckedInterruptDebugData) {
      ++m_bytesReceivedInLastInterrupt;
    }

    if (!m_frameSynced) {
      m_state = State::Syncing;
      
      if (m_spiBufferIndex == 0) {
        if (rxData == kSyncByte0) {
          m_spiInputBuffer[0] = rxData;
          m_spiBufferIndex = 1;
        }
        // If it's not SyncByte0, we stay at index 0 and keep hunting
      } else if (m_spiBufferIndex == 1) {
        if (rxData == kSyncByte1) {
          m_spiInputBuffer[1] = rxData;
          m_spiBufferIndex = 2;
          m_frameSynced = true; // Lock into the frame transmission
        } else {
          // We saw SyncByte0, but the next byte was NOT SyncByte1.
          // This is a verified sync failure.
          ++m_syncDropCount; 
          ++m_bytesLostSyncing;
          
          // Re-evaluate this failed byte. If it happens to be another SyncByte0,
          // keep our index at 1. Otherwise, reset back to hunting.
          if (rxData == kSyncByte0) {
            m_spiInputBuffer[0] = rxData;
            m_spiBufferIndex = 1;
          } else {
            m_spiBufferIndex = 0;
          }
        }
      }
    } else {
      // TRANSCEIVING STATE: Rely purely on counting bytes until the packet finishes
      m_state = State::Transceiving;
      m_spiInputBuffer[m_spiBufferIndex] = rxData;
      ++m_spiBufferIndex;

      if (m_spiBufferIndex >= m_frameSize) {
        // Entire packet size reached. Validate the Fletcher16 checksum.
        validateAndDispatchFrame(); 
        
        // Reset state pointers for the next payload hunt
        m_spiBufferIndex = 0;
        m_frameSynced = false; 
      }
    }
  }

  if (LPSPI4_SR & LPSPI_SR_TDF) {
    uint8_t txIdx = static_cast<uint8_t>(m_spiBufferIndex + 1);
    if (txIdx >= m_frameSize) {
      memcpy((void*)m_spiOutputBuffer, (const void*)m_nextTxFrame, m_frameSize);
      txIdx = 0;
    }
    LPSPI4_TDR = m_spiOutputBuffer[txIdx];
  }

  if (LPSPI4_SR & LPSPI_SR_TEF) {
    LPSPI4_SR = LPSPI_SR_TEF;
    ++m_txErrorCount;
  }

  m_uncheckedInterruptDebugData = true;

  if (LPSPI4_SR & LPSPI_SR_FCF) {
    LPSPI4_SR = LPSPI_SR_FCF;
    ++m_fcfsReceived;
  }

  if (LPSPI4_SR & LPSPI_SR_WCF) {
    LPSPI4_SR = LPSPI_SR_WCF;
  }
}
