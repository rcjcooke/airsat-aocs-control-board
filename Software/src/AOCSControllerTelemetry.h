#ifndef AOCS_CONTROLLER_TELEMETRY_H
#define AOCS_CONTROLLER_TELEMETRY_H
#include <Arduino.h>

struct AOCSControllerTelemetry {

  // AOCS Controller State
  float aocsTargetTorqueNM = 0.0f;

  // Reaction Wheel State
  uint8_t wheelMode = 0;
  uint8_t wheelFault = 0;
  float wheelStoredAngularMomentumKGM2S = 0.0f;
  float wheelTargetAccelerationRADSS = 0.0f;

  // OBC Connection State
  bool obcConnected = false;               // True if OBC link is active
  uint8_t obcCommandsRXCount = 0;          // Count of commands received
  uint8_t obcSyncDropCount = 0;            // Count of sync drops
  uint8_t obcRxErrorCount = 0;             // Count of RX errors
  uint32_t obcTotalBytesReceived = 0;      // Total bytes received from OBC
  uint8_t obcNoOpCount = 0;               // Count of No-Op frames received from OBC

  // Thrusters State
  float thrusters1ThrustN = 0.0f;
  float thrusters2ThrustN = 0.0f;
  float thrusters3ThrustN = 0.0f;
  float thrusters4ThrustN = 0.0f;
  float thrustersPropellantRemainingKg = 0.0f;

};

#endif // AOCS_CONTROLLER_TELEMETRY_H
