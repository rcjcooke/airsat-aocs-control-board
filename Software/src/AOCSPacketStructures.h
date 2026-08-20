#ifndef AIRSAT_AOCS_PACKET_STRUCTURES_H
#define AIRSAT_AOCS_PACKET_STRUCTURES_H

#include <Arduino.h>

namespace AOCSPacketConstants {
    static constexpr uint8_t kSyncByte0 = 0xAA;
    static constexpr uint8_t kSyncByte1 = 0x55;
    static constexpr uint8_t kSyncSize = 2;
    static constexpr uint8_t kChecksumSize = 2;
    static constexpr uint8_t kCommandPayloadSize = 22;
    static constexpr uint8_t kTelemetryPayloadSize = 22;
    static constexpr uint8_t kPacketSize = kSyncSize + kCommandPayloadSize + kChecksumSize;
    static constexpr uint8_t kCommandPacket = 0x11;
    static constexpr uint8_t kNoOpPacket = 0x22;
}

#pragma pack(push, 1)
struct CommandPayload {
    float torque; // N.m
    float thrust[4]; // N
    uint8_t flags;
    uint8_t alignment_pad;
}; // 22 bytes

struct TelemetryPayload {
    float storedAngularMomentum; // kg.m^2/s
    uint16_t propellant; // kg
    uint16_t error_count;
    uint8_t padding[14];
}; // 22 bytes

struct CommandPacket {
    uint8_t sync[2]; // 0xAA, 0x55
    CommandPayload payload;
    uint16_t checksum;
}; // 26 bytes

struct TelemetryPacket {
    uint8_t sync[2]; // 0xAA, 0x55
    TelemetryPayload payload;
    uint16_t checksum;
}; // 26 bytes
#pragma pack(pop)

#endif // AIRSAT_AOCS_PACKET_STRUCTURES_H
