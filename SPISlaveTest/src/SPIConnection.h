#ifndef SPI_CONNECTION_H
#define SPI_CONNECTION_H

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

class SPIConnection {
 public:
  static constexpr uint8_t kDefaultFrameSize = 26;
  static constexpr uint8_t kMaxFrameSize = 64;

  using FrameReadyHandler = void (*)(const uint8_t* frame, uint8_t frameSize);

  enum class State : uint8_t {
    Idle,
    Syncing,
    Transceiving
  };

  struct Stats {
    uint32_t interruptCalls;
    uint32_t lastByteReceived;
    uint32_t totalBytesReceived;
    uint32_t totalFramesReceived;
    uint32_t bytesLostSyncing;
    uint32_t partialFrameErrorCount;
    uint32_t bytesReceivedInLastInterrupt;
    uint32_t fcfsReceived;
    uint32_t txErrorCount;
  };

  explicit SPIConnection(bool debugEnabled, uint8_t frameSize = kDefaultFrameSize);

  void begin();
  void setFrameReadyHandler(FrameReadyHandler handler);
  void setNextTxFrame(const uint8_t* frame, size_t frameSize);
  void printRuntimeDebug();

  uint8_t frameSize() const;
  State state() const;
  Stats statsSnapshot() const;

 private:
  static constexpr uint8_t kSyncByte0 = 0xAA;
  static constexpr uint8_t kSyncByte1 = 0x55;

  static void handleMessageISR();
  void handleMessage();

  void printTCRRegisterDetail() const;
  void printSRRegisterDetail() const;
  void printFSRRegisterDetail() const;
  void printRSRRegisterDetail() const;
  void printSPIParameterRegister() const;
  void printFCRRegisterDetail() const;
  void printIERRegisterDetail() const;
  void printCFGR1RegisterDetail() const;
  void spiRegisterAudit() const;

  bool m_debugEnabled;
  uint8_t m_frameSize;
  volatile State m_state;

  uint8_t m_spiInputBuffer[kMaxFrameSize];
  uint8_t m_spiOutputBuffer[kMaxFrameSize];
  uint8_t m_spiBufferIndex;

  bool m_frameSynced;
  volatile uint8_t m_lastRxFrame[kMaxFrameSize];
  volatile bool m_frameReady;

  uint8_t m_nextTxFrame[kMaxFrameSize];

  volatile uint32_t m_interruptCalls;
  volatile uint32_t m_lastByteReceived;
  volatile uint32_t m_totalBytesReceived;
  volatile uint32_t m_totalFramesReceived;
  volatile uint32_t m_bytesLostSyncing;
  volatile uint32_t m_partialFrameErrorCount;
  volatile uint32_t m_bytesReceivedInLastInterrupt;
  volatile bool m_uncheckedInterruptDebugData;
  volatile uint32_t m_fcfsReceived;
  volatile uint32_t m_txErrorCount;

  FrameReadyHandler m_frameReadyHandler;

  static SPIConnection* s_instance;
};

#endif
