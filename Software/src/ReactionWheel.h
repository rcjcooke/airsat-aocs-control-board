#ifndef AIRSAT_REACTION_WHEEL_H
#define AIRSAT_REACTION_WHEEL_H

#include <Arduino.h>
#include <MoteusTeensy.h>

// Forward declare the types expected by the Moteus library
class ACAN_T4;
class ACAN_T4FD_Settings;

namespace Constants {
    constexpr float WHEEL_INERTIA = 0.0025f; // I (kg*m^2)
    constexpr float RAD_S_TO_HZ = 1.0f / (2.0f * PI);
    constexpr float HZ_TO_RAD_S = 2.0f * PI;
    constexpr float MAX_SPEED_HZ = 80.0f;
    constexpr float MAX_SPEED_CAP_HZ = 85.0f;
    constexpr uint32_t CONTROL_PERIOD_MS = 20;
    constexpr uint32_t UPDATE_PERIOD_MS = 10;
    constexpr uint32_t TIMEOUT_MS = 500;
    // Teensy 32-bit floats provide about 7 digits of precision
    constexpr float ACCELERATION_TOLERANCE_MSS = 0.000001f; // Tolerance for acceleration comparison
    constexpr float VELOCITY_TOLERANCE_MSS = 0.000001f; // Tolerance for velocity comparison
}

class ReactionWheel {
public:

  static constexpr bool kDebug = true;

  enum class RWMode : uint8_t {
    kStopped = 0,
    kRunning = 1,
    kFault = 2
  };

  enum class RWFault : uint8_t {
    kNoFault = 0,
    kMoteusFault = 1,
    kCommunicationError = 2
  };

  struct RWStatus {
    float angularVelocityHz = 0.0f;
    float angularMomentumKGM2S = 0.0f;
    RWMode rwMode = RWMode::kStopped;
    RWFault rwFault = RWFault::kNoFault;

    // Expose underlying motor controller state values for diagnostics
    uint8_t motorControllerMode = 0;
    uint8_t motorControllerFaultCode = 0;
  };

  ReactionWheel(ACAN_T4& canHardware, const ACAN_T4FD_Settings& settings, uint8_t moteusID);

  void begin();
  void service();
  void setTargetTorque(float requestedTorqueNm);
  RWStatus status() const;

  float getAngularVelocity() const;
  float getAngularMomentum() const;
  float getTargetTorque() const;
  float getTargetAngularAcceleration() const;

private:
  struct ControlParams {
    float targetVelocity;
    float accelerationLimit;
  };

  void parseMoteusStatus(ReactionWheel::RWStatus& status, const Moteus::Query::Result& v);

  // Underlying hardware
  MoteusTeensyCanFD* m_canBus;
  Moteus* m_moteus;
  ACAN_T4& m_canHardwareRef;
  const ACAN_T4FD_Settings& m_canSettingsRef;
  uint8_t m_moteusID;

  // Control parameters
  ControlParams m_target;
  ControlParams m_lastCommanded;
  float m_targetTorqueNm;
  bool m_noControlTimeManagement; // If true, commands are sent immediately on next update() call, bypassing the control period timer.

  // Status
  RWStatus m_status;
  uint32_t m_lastMessageTime;

};

#endif // AIRSAT_REACTION_WHEEL_H