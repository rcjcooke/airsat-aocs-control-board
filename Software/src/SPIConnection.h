#ifndef SPI_CONNECTION_H
#define SPI_CONNECTION_H

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

class SPIConnection {
 public:
  static constexpr uint8_t kSyncSize = 2;
  static constexpr uint8_t kChecksumSize = 2;
  static constexpr uint8_t kFrameOverhead = kSyncSize + kChecksumSize;
  static constexpr uint8_t kDefaultPayloadSize = 22;
  static constexpr uint8_t kMaxPayloadSize = 60;
  static constexpr uint8_t kMaxFrameSize = kMaxPayloadSize + kFrameOverhead;

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
    uint32_t txErrorCount;
    uint32_t checksumFailureCount;
  };

  explicit SPIConnection(bool debugEnabled, uint8_t payloadSize = kDefaultPayloadSize);

  void begin();
  void activate();
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
  void buildTxFrameFromPayload(const uint8_t* payload, size_t payloadSize);
  void validateAndDispatchFrame();

  volatile bool m_isActive = false;

  bool m_debugEnabled;
  uint8_t m_payloadSize;
  uint8_t m_packetSize;
  volatile State m_state;

  uint8_t m_spiInputBuffer[kMaxFrameSize];
  uint8_t m_spiOutputBuffer[kMaxFrameSize];
  uint8_t m_spiBufferIndex;

  bool m_packetSynced;
  volatile uint8_t m_lastRxPayload[kMaxPayloadSize];

  uint8_t m_nextTxFrame[kMaxFrameSize];

  volatile uint32_t m_byteRxISRCalls;
  volatile uint32_t m_CSRisingISRCalls;
  volatile uint32_t m_lastByteReceived;
  volatile uint32_t m_totalBytesReceived;
  volatile uint32_t m_totalPacketsReceived;
  volatile uint8_t m_checksumFailureCount;
  volatile uint8_t m_syncDropCount;
  volatile uint32_t m_bytesLostSyncing;
  volatile uint32_t m_bytesReceivedInLastInterrupt;
  volatile bool m_uncheckedInterruptDebugData;
  volatile uint32_t m_fcfsReceived;
  volatile uint32_t m_txErrorCount;

  volatile PayloadReadyHandler m_payloadReadyHandler;

  static SPIConnection* s_instance;
};

#endif
