#ifndef OBC_CONNECTION_H
#define OBC_CONNECTION_H

#include <Arduino.h>

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
  uint8_t padding[14];  // FIXED: Expanded from 1 byte to a 14-byte array to match the 22-byte footprint
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

// Exposed high-level C interface functions for main.cpp
void initOBCConnection();
bool isOBCCommandAvailable();
CommandPayload getLatestOBCCommand();
void updateOBCTelemetry(const TelemetryPayload& freshTelem);
uint16_t getOBCRxErrorCount();
uint32_t getOBCTotalBytesReceived();
uint32_t getOBCTotalInterruptsReceived();
uint32_t getOBCCSFallingEdges();
uint32_t getOBCCSLineState();
void getOBCRawRxBufferSnapshot(uint8_t* destinationBuffer, size_t maxBytes);

#endif
