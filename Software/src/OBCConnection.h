// TODO: Add a timeout on the OBC connection
#ifndef OBC_CONNECTION_H
#define OBC_CONNECTION_H

#include <Arduino.h>
#include <atomic>

#include "SPIConnection.h"
#include "AOCSControllerTelemetry.h"

#pragma pack(push, 1)
struct CommandPayload {
  float torque;
  float thrust[4];
  uint8_t flags;         
  uint8_t alignment_pad;  
}; // Total Size: 22 bytes

// TODO: Fix this up with the right data and make it more efficient
struct TelemetryPayload {
  float momentum;       // 4 bytes
  uint16_t propellant;  // 2 bytes
  uint16_t error_count; // 2 bytes
  uint8_t padding[14];
}; // Total Size: 4 + 2 + 2 + 14 = 22 bytes
#pragma pack(pop)

class OBCConnection {
 public:
  static constexpr uint8_t kCommandMailboxDepth = 2;

  explicit OBCConnection(bool spiDebug = false,
                         uint8_t payloadSize = static_cast<uint8_t>(sizeof(CommandPayload)));

  void begin();
  void activateSPI();
  bool hasNewCommand() const;
  CommandPayload takeLatestCommand();
  void updateTelemetry(const AOCSControllerTelemetry& telemetry);

  bool isConnected() const;
  uint32_t rxErrorCount() const;
  uint32_t totalBytesReceived() const;
  uint8_t syncDropCount() const;
  uint8_t commandCount() const;
  uint8_t noOpCount() const;
  uint8_t malformedFrameCount() const;
  void copyLastRxPayload(uint8_t* destinationBuffer, size_t maxBytes) const;
  SPIConnection& spiConnection();

 private:
  static constexpr uint8_t kCommandFrame = 0x11;
  static constexpr uint8_t kNoOpFrame = 0x22;

  static volatile OBCConnection* s_instance;
  static void onPayloadReceivedCallbackISR(const uint8_t* payload, uint8_t payloadSize);

  void onPayloadReceivedISR(const uint8_t* payload, uint8_t payloadSize);
  void queueTelemetryPayload();

  SPIConnection m_spiConnection;
  CommandPayload m_commandMailbox[kCommandMailboxDepth];
  TelemetryPayload m_telemetryPayload;
  std::atomic<uint8_t> m_commandReadIndex;
  std::atomic<uint8_t> m_commandWriteIndex;

  std::atomic<bool> m_newCommandReady;
  std::atomic<uint32_t> m_commandCount;
  std::atomic<uint32_t> m_noOpCount;
  std::atomic<uint32_t> m_malformedFrame;
};

#endif
