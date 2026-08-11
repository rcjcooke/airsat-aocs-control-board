#ifndef SPI_CONNECTION_H
#define SPI_CONNECTION_H

#include <Arduino.h>
#include <atomic>
#include <stddef.h>
#include <stdint.h>

class SPIConnection {
 public:
  static constexpr uint8_t kSyncSize = 2;
  static constexpr uint8_t kChecksumSize = 2;
  static constexpr uint8_t kFrameOverhead = kSyncSize + kChecksumSize;
  static constexpr uint8_t kDefaultPayloadSize = 22;
  static constexpr uint8_t kMaxPayloadSize = 60;
  static constexpr uint8_t kRxInterruptWatermark = 7; // RDF when RX FIFO has >= 8 bytes
  static constexpr uint8_t kMaxFrameSize = kMaxPayloadSize + kFrameOverhead;
  static constexpr uint8_t kTxFifoHeadroom = 1;
  static constexpr uint8_t kFrameMailboxDepth = 2;
  static constexpr uint8_t kTxFrameBufferDepth = 2;

  using PayloadReadyHandler = void (*)(const uint8_t* payload, uint8_t payloadSize);

  enum class State : uint8_t {
    Idle,
    Syncing,
    Transceiving
  };

  struct Stats {
    uint32_t byteRxISRCalls;
    uint32_t csRisingISRCalls;
    uint32_t lastByteReceived;
    uint32_t totalBytesReceived;
    uint32_t totalPacketsReceived;
    uint32_t bytesLostSyncing;
    uint32_t bytesReceivedInLastInterrupt;
    uint32_t fcfsReceived;
    uint32_t refsReceived;
    uint32_t txErrorCount;
    uint32_t checksumFailureCount;
    uint32_t completedFrameDropCount;
  };

  explicit SPIConnection(bool debugEnabled, uint8_t payloadSize = kDefaultPayloadSize);

  void begin();
  void activate();
  void service();
  void setPayloadReadyHandler(PayloadReadyHandler handler);
  void setNextTxPayload(const uint8_t* payload, size_t payloadSize);
  void copyLastRxPayload(uint8_t* destination, size_t maxBytes) const;
  void copyOutgoingTxFrame(uint8_t* destination, size_t maxBytes) const;

  uint8_t payloadSize() const;
  uint8_t frameSize() const;
  uint32_t totalBytesReceived() const;
  uint8_t syncDropCount() const;
  uint32_t checksumFailureCount() const;
  
  Stats statsSnapshot() const;
  State state() const;

  void printTCRRegisterDetail() const;
  void printSRRegisterDetail() const;
  void printFSRRegisterDetail() const;
  void printRSRRegisterDetail() const;
  void printSPIParameterRegister() const;
  void printFCRRegisterDetail() const;
  void printIERRegisterDetail() const;
  void printCFGR1RegisterDetail() const;
  void spiRegisterAudit() const;

 private:
  static constexpr uint8_t kSyncByte0 = 0xAA;
  static constexpr uint8_t kSyncByte1 = 0x55;

  static uint16_t calculateFletcher16(const uint8_t* data, size_t count);

  static void handleMessageISR();
  void handleMessage();
  void processReceivedByte(uint8_t rxData);
  void primeTxFIFO();
  void buildTxFrameFromPayload(const uint8_t* payload, size_t payloadSize, uint8_t* frameBuffer);
  void validateAndDispatchFrame(const uint8_t* frameData);

  std::atomic<bool> m_isActive{false};

  bool m_debugEnabled;
  uint8_t m_payloadSize;
  uint8_t m_packetSize;
  std::atomic<State> m_state;

  uint8_t m_spiInputBuffer[kMaxFrameSize];
  uint8_t m_txFrameBuffers[kTxFrameBufferDepth][kMaxFrameSize];
  uint8_t m_completedFrameBuffers[kFrameMailboxDepth][kMaxFrameSize];
  uint8_t m_spiBufferIndex;
  uint8_t m_txFrameProgress;
  std::atomic<uint8_t> m_txActiveFrameIndex;
  std::atomic<uint8_t> m_txPendingFrameIndex;
  std::atomic<bool> m_txPendingFrameReady;
  std::atomic<uint8_t> m_completedFrameReadIndex;
  std::atomic<uint8_t> m_completedFrameWriteIndex;
  std::atomic<bool> m_completedFrameReady;

  bool m_packetSynced;
  uint8_t m_lastRxPayloadBuffers[kFrameMailboxDepth][kMaxPayloadSize];
  std::atomic<uint8_t> m_lastRxPayloadIndex;

  std::atomic<uint32_t> m_byteRxISRCalls;
  std::atomic<uint32_t> m_CSRisingISRCalls;
  std::atomic<uint32_t> m_lastByteReceived;
  std::atomic<uint32_t> m_totalBytesReceived;
  std::atomic<uint32_t> m_totalPacketsReceived;
  std::atomic<uint32_t> m_checksumFailureCount;
  std::atomic<uint32_t> m_syncDropCount;
  std::atomic<uint32_t> m_bytesLostSyncing;
  std::atomic<uint32_t> m_bytesReceivedInLastInterrupt;
  std::atomic<bool> m_uncheckedInterruptDebugData;
  std::atomic<uint32_t> m_fcfsReceived;
  std::atomic<uint32_t> m_refsReceived;
  std::atomic<uint32_t> m_txErrorCount;
  std::atomic<uint32_t> m_completedFrameDropCount;

  std::atomic<PayloadReadyHandler> m_payloadReadyHandler;

  static SPIConnection* s_instance;
};

#endif
