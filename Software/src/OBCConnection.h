#ifndef OBC_CONNECTION_H
#define OBC_CONNECTION_H

#include <Arduino.h>

#pragma pack(push, 1)
struct CommandPayload {
  float torque;
  float thrust[4];
  uint8_t flags;         
  uint8_t alignment_pad;  
};

struct TelemetryPayload {
  float momentum;
  uint16_t propellant;
  uint16_t error_count;
  uint8_t padding; 
};

struct CommandFrame {
  uint8_t sync[2]; 
  CommandPayload payload;
  uint16_t checksum;
};

struct TelemetryFrame {
  uint8_t sync[2]; 
  TelemetryPayload payload;
  uint16_t checksum;
};
#pragma pack(pop)

// Exposed high-level C interface functions for main.cpp
void initOBCConnection();
bool isOBCCommandAvailable();
CommandPayload getLatestOBCCommand();
void updateOBCTelemetry(const TelemetryPayload& freshTelem);
uint16_t getOBCRxErrorCount();

#endif
