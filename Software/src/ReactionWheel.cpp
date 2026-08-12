#include "ReactionWheel.h"
#include <ACAN_T4.h>

ReactionWheel::ReactionWheel(ACAN_T4& canHardware, const ACAN_T4FD_Settings& settings, uint8_t moteusID)
  : m_canBus(nullptr), 
    m_moteus(nullptr),
    m_canHardwareRef(canHardware),
    m_canSettingsRef(settings),
    m_moteusID(moteusID),
    m_target(), 
    m_lastCommanded(), 
    m_targetTorqueNm(0.0f),
    m_noControlTimeManagement(true), 
    m_status(), 
    m_lastMessageTime(0) {}

void ReactionWheel::begin() {
  // Construct the CAN interface
  m_canBus = new MoteusTeensyCanFD(m_canHardwareRef, m_canSettingsRef);

  // Construct the Moteus instance
  Moteus::Options options;
  options.id = m_moteusID;
  m_moteus = new Moteus(*m_canBus, options);

  // Test contact with the physical controller
  bool success = m_moteus->SetStop(); 
  if (success) {
    m_lastMessageTime = millis();
    m_status.rwMode = RWMode::kRunning;
    m_status.rwFault = RWFault::kNoFault;
  } else {
    // Got no reply from the Moteus controller, so set the fault state
    m_status.rwMode = RWMode::kFault;
    m_status.rwFault = RWFault::kCommunicationError;
  }
}

void ReactionWheel::setTargetTorque(float requestedTorqueNm) {
  m_targetTorqueNm = requestedTorqueNm;

  float targetAlphaRadSS = requestedTorqueNm / Constants::WHEEL_INERTIA;
  float rawTargetVelocityHz = (requestedTorqueNm >= 0.0f) 
                              ? Constants::MAX_SPEED_HZ 
                              : -Constants::MAX_SPEED_HZ;
                              
  m_target.accelerationLimit = abs(targetAlphaRadSS * Constants::RAD_S_TO_HZ);
  m_target.targetVelocity = constrain(rawTargetVelocityHz, -Constants::MAX_SPEED_HZ, Constants::MAX_SPEED_HZ);

}

float ReactionWheel::getAngularVelocity() const { 
  return m_status.angularVelocityHz;
}

float ReactionWheel::getAngularMomentum() const {
  return m_status.angularMomentumKGM2S;
}

float ReactionWheel::getTargetAngularAcceleration() const {
  return m_target.accelerationLimit * Constants::HZ_TO_RAD_S;
}

float ReactionWheel::getTargetTorque() const {
  return m_targetTorqueNm;
}

ReactionWheel::RWStatus ReactionWheel::status() const {
  return m_status;
}

void ReactionWheel::update() {

  // Just in case
  if (m_moteus == nullptr) return; 

  static uint32_t sendTimer = 0;
  static uint32_t pollTimer = 0;
  
  // Only send a new command if it has been long enough since the last command (unless we're not managing that)
  if (m_noControlTimeManagement || millis() - sendTimer >= Constants::CONTROL_PERIOD_MS) { 
    sendTimer = millis();

    auto isEqualWithinTolerance = [](float a, float b, float tolerance) {
      return fabs(a - b) <= tolerance;
    };

    // Check to see whether the latest target is different within the controllers precision
    if (!isEqualWithinTolerance(m_target.targetVelocity, m_lastCommanded.targetVelocity, Constants::VELOCITY_TOLERANCE_MSS) || 
        !isEqualWithinTolerance(m_target.accelerationLimit, m_lastCommanded.accelerationLimit, Constants::ACCELERATION_TOLERANCE_MSS)) {

      // Update the last commanded values
      m_lastCommanded.targetVelocity = m_target.targetVelocity;
      m_lastCommanded.accelerationLimit = m_target.accelerationLimit;
      
      // Use the Moteus acceleration limit for AirSat torque control
      Moteus::PositionMode::Command cmd;
      cmd.position = std::numeric_limits<double>::quiet_NaN();
      cmd.velocity = m_lastCommanded.targetVelocity;
      cmd.velocity_limit = Constants::MAX_SPEED_CAP_HZ; 
      cmd.accel_limit = m_lastCommanded.accelerationLimit;

      // Using Begin rather than Set to avoid blocking calls.
      m_moteus->BeginPosition(cmd);
      
    }
  }

  // Get the latest status from the motor controller, but only if it has been long enough since the last poll
  if (millis() - pollTimer >= Constants::UPDATE_PERIOD_MS) { 
    pollTimer = millis();

    // Get the current motor status
    if (m_moteus->Poll()) {
      m_lastMessageTime = millis();
      const Moteus::Query::Result v = m_moteus->last_result().values;
      parseMoteusStatus(m_status, v);
    } else {
      m_status.rwMode = RWMode::kFault;
      m_status.rwFault = RWFault::kCommunicationError;
    }
  }

  // Backup hardware timing link dropout watchdog
  if (millis() - m_lastMessageTime > Constants::TIMEOUT_MS) { 
    m_status.rwMode = RWMode::kFault;
    m_status.rwFault = RWFault::kCommunicationError;
  }
}

void ReactionWheel::parseMoteusStatus(RWStatus& status, const Moteus::Query::Result& v) {
  status.angularVelocityHz = v.velocity;
  status.angularMomentumKGM2S = Constants::WHEEL_INERTIA * v.velocity * Constants::HZ_TO_RAD_S;
  status.motorControllerMode = static_cast<uint8_t>(v.mode);
  status.motorControllerFaultCode = v.fault;
  if (v.mode == mjbots::moteus::Mode::kFault) {
    status.rwMode = RWMode::kFault;
    status.rwFault = RWFault::kMoteusFault;
  } else {
    status.rwMode = RWMode::kRunning;
    status.rwFault = RWFault::kNoFault;
  }
}

