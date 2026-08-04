#ifndef OBC_CONNECTION_H
#define OBC_CONNECTION_H

#include <Arduino.h>

#include "SPIConnection.h"

#pragma pack(push, 1)
struct CommandPayload {
  float torque;
  float thrust[4];
  uint8_t flags;         
  uint8_t alignment_pad;  
}; // Total Size: 22 bytes

struct TelemetryPayload {
  float momentum;       // 4 bytes
  uint16_t propellant;  // 2 bytes
  uint16_t error_count; // 2 bytes
  uint8_t padding[14];
}; // Total Size: 4 + 2 + 2 + 14 = 22 bytes
#pragma pack(pop)

class OBCConnection {
 public:
  explicit OBCConnection(bool spiDebug = false,
                         uint8_t payloadSize = static_cast<uint8_t>(sizeof(CommandPayload)));

  void initialize();
  bool hasNewCommand() const;
  CommandPayload takeLatestCommand();
  void updateTelemetry(const TelemetryPayload& telemetry);

  uint16_t rxErrorCount() const;
  uint32_t totalBytesReceived() const;
  uint32_t totalInterruptsReceived() const;
  uint32_t syncDropCount() const;
  uint32_t commandCount() const;
  uint32_t noOpCount() const;
  void copyLastRxPayload(uint8_t* destinationBuffer, size_t maxBytes) const;

 private:
  static constexpr uint8_t kCommandFrame = 0x11;
  static constexpr uint8_t kNoOpFrame = 0x22;

  static OBCConnection* s_instance;
  static void onPayloadReceivedCallback(const uint8_t* payload, uint8_t payloadSize);

  void onPayloadReceived(const uint8_t* payload, uint8_t payloadSize);
  void queueTelemetryPayload();

  SPIConnection m_spiConnection;
  volatile bool m_newCommandReady;
  CommandPayload m_verifiedCommand;
  TelemetryPayload m_telemetryPayload;
  uint32_t m_commandCount;
  uint32_t m_noOpCount;
};

#endif
