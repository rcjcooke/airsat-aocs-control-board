#ifndef AIRSAT_REACTION_WHEEL_H
#define AIRSAT_REACTION_WHEEL_H

#include <Arduino.h>
#include <MoteusTeensy.h>

// Forward declare the types expected by the Moteus library
class ACAN_T4;
class ACAN_T4FD_Settings;

namespace Constants {
  // Conversion factors
  constexpr float RAD_S_TO_HZ = 1.0f / (2.0f * PI);
  constexpr float HZ_TO_RAD_S = 2.0f * PI;
  // Physical constants and limits
  constexpr float STATOR_INERTIA = 6.7464E-06f; // I (kg*m^2)
  constexpr float FLYWHEEL_INERTIA = 0.000215f; // I (kg*m^2)
  constexpr float WHEEL_INERTIA = STATOR_INERTIA + FLYWHEEL_INERTIA; // I (kg*m^2)
  constexpr float MAX_MOTOR_TORQUE_NM = 1.5f; // Maximum torque (Nm) - actually 1.7, but limited for safety
  constexpr float MAX_MOTOR_SPEED_HZ = 83.0f; // 4980 RPM (Measured limit - losses because of 22.2V supply, 330Kv motor, FOC control and electrical losses)
  constexpr float MAX_MOTOR_ACCELERATION_HZ = MAX_MOTOR_TORQUE_NM / WHEEL_INERTIA * RAD_S_TO_HZ; // Maximum acceleration (Hz/s)
  // Control constants
  constexpr uint32_t CONTROL_PERIOD_MS = 20;
  constexpr uint32_t TIMEOUT_MS = 500;
  // Teensy 32-bit floats provide about 7 digits of precision
  constexpr float ACCELERATION_TOLERANCE_MSS = 0.000001f; // Tolerance for acceleration comparison
  constexpr float VELOCITY_TOLERANCE_MSS = 0.000001f; // Tolerance for velocity comparison
} // namespace Constants

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
    kCommunicationError = 2,
    kCANTimeout = 3 // Timeout on the moteus controller waiting for the next CAN command
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

  ReactionWheel(uint8_t silentModePin, ACAN_T4& canHardware, const ACAN_T4FD_Settings& settings, uint8_t moteusID);

  void begin();
  void service();
  void setTargetTorque(float requestedTorqueNm);
  RWStatus status() const;

  float getAngularVelocity() const;
  float getAngularMomentum() const;
  float getTargetTorque() const;

private:
  inline static const Moteus::PositionMode::Format kPositionFormat = [] {
    Moteus::PositionMode::Format format;
    format.position = mjbots::moteus::Resolution::kFloat;
    format.velocity = mjbots::moteus::Resolution::kFloat;
    format.kp_scale = mjbots::moteus::Resolution::kFloat;
    format.kd_scale = mjbots::moteus::Resolution::kFloat;
    format.ilimit_scale = mjbots::moteus::Resolution::kFloat;
    format.feedforward_torque = mjbots::moteus::Resolution::kFloat;
    format.maximum_torque = mjbots::moteus::Resolution::kFloat;
    return format;
  }();

  void parseMoteusStatus(ReactionWheel::RWStatus& status, const Moteus::Query::Result& v);

  // Underlying hardware
  uint8_t m_silentModePin;
  MoteusTeensyCanFD* m_canBus;
  Moteus* m_moteus;
  ACAN_T4& m_canHardwareRef;
  const ACAN_T4FD_Settings& m_canSettingsRef;
  uint8_t m_moteusID;

  // Control parameters
  float m_targetTorqueNm;
  float m_lastCommandedTorqueNm;
  bool m_noControlTimeManagement; // If true, commands are sent immediately on next update() call, bypassing the control period timer.

  // Status
  RWStatus m_status;
  uint32_t m_lastMessageTime;

};

#endif // AIRSAT_REACTION_WHEEL_H