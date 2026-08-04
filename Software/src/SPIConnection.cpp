#include "SPIConnection.h"

#include <SPISlave_T4.h>

#include <string.h>

namespace {
SPISlave_T4<&SPI, SPI_8_BITS> g_spi;

uint8_t clampFrameSize(uint8_t requestedFrameSize) {
  if (requestedFrameSize < 2) {
    return 2;
  }
  if (requestedFrameSize > SPIConnection::kMaxFrameSize) {
    return SPIConnection::kMaxFrameSize;
  }
  return requestedFrameSize;
}
}  // namespace

SPIConnection* SPIConnection::s_instance = nullptr;

SPIConnection::SPIConnection(bool debugEnabled, uint8_t frameSize)
    : m_debugEnabled(debugEnabled),
      m_frameSize(clampFrameSize(frameSize)),
      m_state(State::Idle),
      m_spiInputBuffer{0},
      m_spiOutputBuffer{0},
      m_spiBufferIndex(0),
      m_frameSynced(false),
      m_lastRxFrame{0},
      m_frameReady(false),
      m_nextTxFrame{0},
      m_interruptCalls(0),
      m_lastByteReceived(0),
      m_totalBytesReceived(0),
      m_totalFramesReceived(0),
      m_bytesLostSyncing(0),
      m_partialFrameErrorCount(0),
      m_bytesReceivedInLastInterrupt(0),
      m_uncheckedInterruptDebugData(false),
      m_fcfsReceived(0),
      m_txErrorCount(0),
      m_frameReadyHandler(nullptr) {}

void SPIConnection::begin() {
  s_instance = this;

  m_state = State::Idle;
  m_spiBufferIndex = 0;
  m_frameSynced = false;
  m_frameReady = false;
  m_uncheckedInterruptDebugData = false;

  memcpy((void*)m_spiOutputBuffer, (const void*)m_nextTxFrame, m_frameSize);

  g_spi.setIERTriggerMode(true, false);
  g_spi.begin();
  g_spi.onReceive(handleMessageISR);

  if (m_debugEnabled) {
    Serial.println("[SPI] Transport initialised");
    Serial.flush();
  }
}

void SPIConnection::setFrameReadyHandler(FrameReadyHandler handler) {
  noInterrupts();
  m_frameReadyHandler = handler;
  interrupts();
}

void SPIConnection::setNextTxFrame(const uint8_t* frame, size_t frameSize) {
  if (frame == nullptr) {
    return;
  }

  const size_t boundedSize = (frameSize < m_frameSize) ? frameSize : m_frameSize;

  noInterrupts();
  memset(m_nextTxFrame, 0, sizeof(m_nextTxFrame));
  memcpy(m_nextTxFrame, frame, boundedSize);
  interrupts();
}

void SPIConnection::copyLastRxFrame(uint8_t* destination, size_t maxBytes) const {
  if (destination == nullptr || maxBytes == 0) {
    return;
  }

  const size_t boundedSize = (maxBytes < m_frameSize) ? maxBytes : m_frameSize;

  noInterrupts();
  memcpy(destination, (const void*)m_lastRxFrame, boundedSize);
  interrupts();
}

uint8_t SPIConnection::frameSize() const {
  return m_frameSize;
}

SPIConnection::Stats SPIConnection::statsSnapshot() const {
  noInterrupts();
  const Stats snapshot = {
      m_interruptCalls,
      m_lastByteReceived,
      m_totalBytesReceived,
      m_totalFramesReceived,
      m_bytesLostSyncing,
      m_partialFrameErrorCount,
      m_bytesReceivedInLastInterrupt,
      m_fcfsReceived,
      m_txErrorCount};
  interrupts();
  return snapshot;
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
      if (m_spiBufferIndex == 0 && rxData == kSyncByte0) {
        m_spiInputBuffer[m_spiBufferIndex] = rxData;
        m_spiBufferIndex = 1;
      } else if (m_spiBufferIndex == 1 && rxData == kSyncByte1) {
        m_spiInputBuffer[m_spiBufferIndex] = rxData;
        m_spiBufferIndex = 2;
        m_frameSynced = true;
      } else {
        ++m_bytesLostSyncing;
        m_spiBufferIndex = 0;
      }
    } else {
      m_state = State::Transceiving;
      const bool syncMismatch =
          (m_spiBufferIndex == 0 && rxData != kSyncByte0) ||
          (m_spiBufferIndex == 1 && rxData != kSyncByte1);

      if (syncMismatch) {
        m_frameSynced = false;
        ++m_bytesLostSyncing;
        m_spiBufferIndex = 0;
        ++m_partialFrameErrorCount;
      } else {
        m_spiInputBuffer[m_spiBufferIndex] = rxData;
        ++m_spiBufferIndex;

        if (m_spiBufferIndex >= m_frameSize) {
          memcpy((void*)m_lastRxFrame, (const void*)m_spiInputBuffer, m_frameSize);
          m_frameReady = true;

          if (m_frameReadyHandler != nullptr) {
            m_frameReadyHandler((const uint8_t*)m_lastRxFrame, m_frameSize);
          }

          m_spiBufferIndex = 0;
          ++m_totalFramesReceived;
        }
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
