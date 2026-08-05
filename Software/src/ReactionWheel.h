#pragma once
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
    constexpr uint32_t TIMEOUT_MS = 500;
}

class ReactionWheel {
public:
  ReactionWheel(ACAN_T4& canHardware, const ACAN_T4FD_Settings& settings, uint8_t moteusID);

  void begin();
  void update();
  void setTargetTorque(float requestedTorqueNm);

  float getAngularVelocity() const;
  float getAngularMomentum() const;
  uint8_t getModeState() const;
  uint8_t getFaultCode() const;
  bool isConnected() const;

private:
  MoteusTeensyCanFD   _canBus;
  Moteus              _moteus;

  float    _targetVelocity;
  float    _accelLimit;
  uint32_t _lastMessageTime;
  bool     _connected;
};
