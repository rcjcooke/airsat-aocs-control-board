#ifndef OBC_CONNECTION_H
#define OBC_CONNECTION_H

#include <Arduino.h>

#pragma pack(push, 1)
// Raw structural payloads
struct CommandPayload {
  float torque;
  float thrust[4];
};

struct TelemetryPayload {
  float momentum;
  uint16_t propellant;
  uint16_t error_count;
  uint8_t padding[14]; // Explicitly pads payload to 22 bytes
};

// Full 26-byte Wire Frames
struct CommandFrame {
  uint8_t sync[2]; // [0] = 0xAA, [1] = 0x55
  CommandPayload payload;
  uint16_t checksum;
};

struct TelemetryFrame {
  uint8_t sync[2]; // [0] = 0xAA, [1] = 0x55
  TelemetryPayload payload;
  uint16_t checksum;
};
#pragma pack(pop)

class OBCConnection {
public:
  OBCConnection();
  void begin();
  
  // High-level API for main.cpp
  bool isCommandAvailable();
  CommandPayload getLatestCommand();
  void updateTelemetry(const TelemetryPayload& newTelem);
  uint16_t getRxErrorCount() const;

  // Internal ISR handler (must be public to be accessible by global wrapper)
  void handleInterrupt();

private:
  uint16_t calculateFletcher16(const uint8_t* data, size_t count);
  void updateTxBuffer();

  // Internal double buffering or frame storage
  volatile CommandFrame _incomingFrame;
  volatile TelemetryFrame _outgoingFrame;
  
  // Storage for processed, verified commands passed to main loop
  CommandPayload _verifiedCommand;
  volatile bool _newCommandReady;

  // State machine tracking variables
  volatile uint8_t* _rxPtr;
  volatile uint8_t* _txPtr;
  volatile uint8_t _rxIndex;
  volatile bool _frameSynced;
  volatile uint16_t _localErrorCount;
};

// Global instance pointer needed to bridge C++ object into the hardware ISR callback
extern OBCConnection* g_obcConnectionPtr;

#endif
