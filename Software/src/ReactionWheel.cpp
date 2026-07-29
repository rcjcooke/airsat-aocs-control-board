#include "ReactionWheel.h"
#include <FlexCAN_T4.h>

static FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_256> canFD;

ReactionWheel::ReactionWheel(uint8_t moteusID)
  : _moteusID(moteusID), _targetVelocity(0.0f), _accelLimit(0.0f), _lastMessageTime(0),
    _currentVelocity(0.0f), _modeState(0), _faultCode(0), _connected(false) {}

void ReactionWheel::begin() {
  canFD.begin();
  CANFD_timings_t timings;
  timings.clock = CLK_60MHz;
  timings.baudrate = 1000000;    
  timings.baudrateFD = 5000000;  
  canFD.setBaudRate(timings);
  canFD.setRegions(64);
  canFD.enableMBInterrupts();
}

void ReactionWheel::setTargetTorque(float requestedTorqueNm) {
  // 1. Calculate target angular acceleration (alpha = Torque / Inertia) in rad/s^2
  float targetAlphaRadSS = requestedTorqueNm / Constants::MOMENT_OF_INERTIA;
  
  // 2. Convert acceleration to native Moteus profile tracking unit (Hz/s^2)
  _accelLimit = abs(targetAlphaRadSS * Constants::RAD_S_TO_HZ);
  
  // 3. Set the target velocity direction depending on the torque command sign
  float rawTargetVelocityHz = (requestedTorqueNm >= 0.0f) 
                              ? Constants::MAX_SPEED_HZ 
                              : -Constants::MAX_SPEED_HZ;
                              
  // 4. Bound the velocity to safe physical limits
  _targetVelocity = constrain(rawTargetVelocityHz, -Constants::MAX_SPEED_HZ, Constants::MAX_SPEED_HZ);
}

float ReactionWheel::getAngularVelocity() const { 
  return _currentVelocity; 
}

float ReactionWheel::getAngularMomentum() const {
  // Convert Hz (revolutions per second) back to radians per second
  float velocityRadSS = _currentVelocity * Constants::HZ_TO_RAD_S_TO_HZ;
  
  // L = I * w (kg*m^2/s)
  return Constants::MOMENT_OF_INERTIA * velocityRadSS;
}

uint8_t ReactionWheel::getModeState() const { return _modeState; }
uint8_t ReactionWheel::getFaultCode() const { return _faultCode; }
bool ReactionWheel::isConnected() const { return _connected; }

void ReactionWheel::sendTrajectoryCommand(float velocityHz, float accelLimitHzSS) {
  CANFD_message_t txFrame;
  txFrame.id = _moteusID;
  txFrame.flags.extended = 0; 
  txFrame.edl = 1;            
  txFrame.brs = 1;            
  
  uint8_t idx = 0;
  
  txFrame.buf[idx++] = 0x01; 
  txFrame.buf[idx++] = 0x00; 
  txFrame.buf[idx++] = 0x0A; // Position/Trajectory Mode

  txFrame.buf[idx++] = 0x42; 
  txFrame.buf[idx++] = 0x020; 
  
  float nanPosition = NAN;   
  memcpy(&txFrame.buf[idx], &nanPosition, 4);
  idx += 4;
  memcpy(&txFrame.buf[idx], &velocityHz, 4);
  idx += 4;

  txFrame.buf[idx++] = 0x42; 
  txFrame.buf[idx++] = 0x028; 
  
  float maxVelocityCap = Constants::MAX_SPEED_CAP_HZ; 
  memcpy(&txFrame.buf[idx], &maxVelocityCap, 4);
  idx += 4;
  memcpy(&txFrame.buf[idx], &accelLimitHzSS, 4);
  idx += 4;

  txFrame.buf[idx++] = 0x14; 
  txFrame.buf[idx++] = 0x04; 
  txFrame.buf[idx++] = 0x00; 

  txFrame.len = idx;
  canFD.write(txFrame); 
}

void ReactionWheel::update() {
  static uint32_t sendTimer = 0;
  if (millis() - sendTimer >= Constants::CONTROL_PERIOD_MS) { 
    sendTimer = millis();
    sendTrajectoryCommand(_targetVelocity, _accelLimit);
  }

  CANFD_message_t rxFrame;
  while (canFD.read(rxFrame)) {
    if (rxFrame.id == (uint32_t)(_moteusID | 0x8000)) { 
      _lastMessageTime = millis();
      _connected = true;

      uint8_t idx = 0;
      while (idx < rxFrame.len) {
        uint8_t cmdByte = rxFrame.buf[idx++];
        
        if (cmdByte == 0x24) { 
          uint8_t numRegs = rxFrame.buf[idx++];
          uint8_t startReg = rxFrame.buf[idx++];
          
          if (startReg == 0x000 && numRegs >= 4) {
            int16_t rawMode = (rxFrame.buf[idx+1] << 8) | rxFrame.buf[idx];
            _modeState = (uint8_t)rawMode;
            idx += 4; 
            
            int16_t rawVelocity = (rxFrame.buf[idx+1] << 8) | rxFrame.buf[idx];
            _currentVelocity = (float)rawVelocity * 0.0001f; 
            idx += 4; 
          }
        } 
        else if (cmdByte == 0x21) { 
          uint8_t reg = rxFrame.buf[idx++];
          if (reg == 0x00F) {
            _faultCode = rxFrame.buf[idx++];
          } else {
            idx++;
          }
        }
        else {
          break; 
        }
      }
    }
  }

  if (millis() - _lastMessageTime > Constants::TIMEOUT_MS) { 
    _connected = false;
  }
}
