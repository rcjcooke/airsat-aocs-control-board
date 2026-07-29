#ifndef REACTION_WHEEL_H
#define REACTION_WHEEL_H

#include <Arduino.h>

class ReactionWheel {
public:
  struct Constants {
    static constexpr float MAX_SPEED_HZ      = 80.0f;     
    static constexpr float MAX_SPEED_CAP_HZ  = 85.0f;     
    static constexpr float MOMENT_OF_INERTIA = 0.0025f;   // I (kg*m^2)
    static constexpr float HZ_TO_RAD_S_TO_HZ = (2.0f * PI);
    static constexpr float RAD_S_TO_HZ       = 1.0f / HZ_TO_RAD_S_TO_HZ;
    static constexpr uint32_t TIMEOUT_MS     = 500;       
    static constexpr uint32_t CONTROL_PERIOD_MS = 20;     
  };

  ReactionWheel(uint8_t moteusID = 1);
  
  void begin();
  void update(); 
  
  void setTargetTorque(float requestedTorqueNm);
  
  float getAngularVelocity() const; 
  // Calculated in metric units (kg*m^2/s)
  float getAngularMomentum() const;
  uint8_t getModeState() const;     
  uint8_t getFaultCode() const;     
  bool isConnected() const;

private:
  void sendTrajectoryCommand(float velocityHz, float accelLimitHzSS);

  uint8_t  _moteusID;
  float    _targetVelocity;
  float    _accelLimit;
  uint32_t _lastMessageTime;
  
  float    _currentVelocity;
  uint8_t  _modeState;
  uint8_t  _faultCode;
  bool     _connected;
};

#endif
