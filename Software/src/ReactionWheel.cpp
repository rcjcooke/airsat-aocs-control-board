#include "ReactionWheel.h"
#include <ACAN_T4.h>

ReactionWheel::ReactionWheel(ACAN_T4& canHardware, const ACAN_T4FD_Settings& settings, uint8_t moteusID)
  : _canBus(canHardware, settings), 
    _moteus(_canBus, [moteusID]() {
      Moteus::Options options;
      options.id = moteusID;
      return options;
    }()),
    _targetVelocity(0.0f), _accelLimit(0.0f), _lastMessageTime(0), _connected(false) {}

void ReactionWheel::begin() {
  // Test contact with the physical controller by sending a Stop instruction
  _connected = _moteus.SetStop(); 
  if (_connected) {
    _lastMessageTime = millis();
  }
}

void ReactionWheel::setTargetTorque(float requestedTorqueNm) {
  float targetAlphaRadSS = requestedTorqueNm / Constants::WHEEL_INERTIA;
  _accelLimit = abs(targetAlphaRadSS * Constants::RAD_S_TO_HZ);
  
  float rawTargetVelocityHz = (requestedTorqueNm >= 0.0f) 
                              ? Constants::MAX_SPEED_HZ 
                              : -Constants::MAX_SPEED_HZ;
                              
  _targetVelocity = constrain(rawTargetVelocityHz, -Constants::MAX_SPEED_HZ, Constants::MAX_SPEED_HZ);
}

float ReactionWheel::getAngularVelocity() const { 
  return _moteus.last_result().values.velocity; 
}

float ReactionWheel::getAngularMomentum() const {
  float velocityRadS = _moteus.last_result().values.velocity * Constants::HZ_TO_RAD_S;
  return Constants::WHEEL_INERTIA * velocityRadS;
}

uint8_t ReactionWheel::getModeState() const { 
  return static_cast<uint8_t>(_moteus.last_result().values.mode); 
}

uint8_t ReactionWheel::getFaultCode() const { 
  return _moteus.last_result().values.fault; 
}

bool ReactionWheel::isConnected() const { 
  return _connected; 
}

void ReactionWheel::update() {
  static uint32_t sendTimer = 0;
  
  if (millis() - sendTimer >= Constants::CONTROL_PERIOD_MS) { 
    sendTimer = millis();

    Moteus::PositionMode::Command cmd;
    cmd.position = NaN;
    cmd.velocity = _targetVelocity;
    cmd.velocity_limit = Constants::MAX_SPEED_CAP_HZ; 
    cmd.accel_limit = _accelLimit;

    // Evaluates direct bus response instantly
    if (_moteus.SetPosition(cmd)) {
      _lastMessageTime = millis();
      _connected = true;
    } else {
      _connected = false;
    }
  }

  // Backup hardware timing link dropout watchdog
  if (millis() - _lastMessageTime > Constants::TIMEOUT_MS) { 
    _connected = false;
  }
}
