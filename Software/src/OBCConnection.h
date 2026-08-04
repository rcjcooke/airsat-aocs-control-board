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

struct CommandFrame {
  uint8_t sync[2]; 
  CommandPayload payload;
  uint16_t checksum;
}; // Total Size: 2 + 22 + 2 = 26 bytes

struct TelemetryFrame {
  uint8_t sync[2]; 
  TelemetryPayload payload;
  uint16_t checksum;
}; // Total Size: 2 + 22 + 2 = 26 bytes
#pragma pack(pop)

class OBCConnection {
 public:
  explicit OBCConnection(bool spiDebug = false,
                         uint8_t frameSize = static_cast<uint8_t>(sizeof(CommandFrame)));

  void initialize();
  bool hasNewCommand() const;
  CommandPayload takeLatestCommand();
  void updateTelemetry(const TelemetryPayload& telemetry);

  uint16_t rxErrorCount() const;
  uint32_t totalBytesReceived() const;
  uint32_t totalInterruptsReceived() const;
  uint32_t csFallingEdges() const;
  uint32_t csLineState() const;
  void copyLastRxFrame(uint8_t* destinationBuffer, size_t maxBytes) const;

 private:
  static constexpr uint8_t kSyncByte0 = 0xAA;
  static constexpr uint8_t kSyncByte1 = 0x55;

  static OBCConnection* s_instance;
  static void onFrameReceivedCallback(const uint8_t* frame, uint8_t frameSize);

  void onFrameReceived(const uint8_t* frame, uint8_t frameSize);
  void refreshTelemetryTxFrame();
  static uint16_t calculateFletcher16(const uint8_t* data, size_t count);

  SPIConnection m_spiConnection;
  volatile bool m_newCommandReady;
  volatile uint16_t m_localErrorCount;
  CommandPayload m_verifiedCommand;
  TelemetryFrame m_outgoingFrame;
};

#endif
