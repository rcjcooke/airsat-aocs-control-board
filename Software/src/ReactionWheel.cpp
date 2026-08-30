#include "ReactionWheel.h"
#include <ACAN_T4.h>

ReactionWheel::ReactionWheel(uint8_t silentModePin, ACAN_T4& canHardware, const ACAN_T4FD_Settings& settings, uint8_t moteusID)
  : m_silentModePin(silentModePin),
    m_canBus(nullptr), 
    m_moteus(nullptr),
    m_canHardwareRef(canHardware),
    m_canSettingsRef(settings),
    m_moteusID(moteusID),
    m_targetTorqueNm(0.0f),
    m_lastCommandedTorqueNm(0.0f), 
    m_noControlTimeManagement(false), 
    m_status(), 
    m_lastMessageTime(0) {}

void ReactionWheel::begin() {
  // Make sure we're not in silent (listen-only) mode on the CAN transceiver
  pinMode(m_silentModePin, OUTPUT);
  digitalWrite(m_silentModePin, LOW);

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
    return; // No point continuing if we can't talk to the controller
  }

  // Send an initial 0 velocity command to kick off the baseline trajectory tracking
  Moteus::PositionMode::Command init_cmd;
  init_cmd.position = std::numeric_limits<float>::quiet_NaN();
  init_cmd.velocity = 0.0;
  init_cmd.kp_scale = 1.0;
  init_cmd.kd_scale = 1.0;
  bool initSuccess = m_moteus->SetPosition(init_cmd, &kPositionFormat);
  if (!initSuccess) {
    // If the formatted frame failed, your service loop will fail too. Flag it here.
    m_status.rwMode = RWMode::kFault;
    m_status.rwFault = RWFault::kCommunicationError;
    if (kDebug) Serial.println("[RW] [ERROR] Formatted initialisation frame rejected!");
  }
  delay(50); // Give the moteus firmware 50ms to configure its internal trajectory anchor
}

void ReactionWheel::setTargetTorque(float requestedTorqueNm) {
  m_targetTorqueNm = constrain(requestedTorqueNm, -Constants::MAX_MOTOR_TORQUE_NM, Constants::MAX_MOTOR_TORQUE_NM);
  if (kDebug) {
    if (Serial) {
      Serial.printf("[RW] [DEBUG] ReactionWheel::setTargetTorque() - Requested torque: %.4f Nm | Constrained torque: %.4f Nm\r\n",
                    requestedTorqueNm,
                    m_targetTorqueNm);
    }
  }
}

float ReactionWheel::getAngularVelocity() const { 
  return m_status.angularVelocityHz;
}

float ReactionWheel::getAngularMomentum() const {
  return m_status.angularMomentumKGM2S;
}

float ReactionWheel::getTargetTorque() const {
  return m_targetTorqueNm;
}

ReactionWheel::RWStatus ReactionWheel::status() const {
  return m_status;
}

void ReactionWheel::service() {

  // Just in case
  if (m_moteus == nullptr) return; 

  static uint32_t sendTimer = 0;
  
  // Only send a new command if it has been long enough since the last command (unless we're not managing that)
  if (m_noControlTimeManagement || millis() - sendTimer >= Constants::CONTROL_PERIOD_MS) { 
    sendTimer = millis();

    // Update the last commanded values
    m_lastCommandedTorqueNm = m_targetTorqueNm;

    // Use the Moteus acceleration limit for AirSat torque control
    // Make sure physical constraints are imposed in case they haven't already been applied
    Moteus::PositionMode::Command cmd;
    cmd.position = std::numeric_limits<float>::quiet_NaN();
    cmd.velocity = 0.0;
    cmd.kp_scale = 0.0;
    cmd.kd_scale = 0.0;
    cmd.ilimit_scale = 0.0;
    cmd.maximum_torque = Constants::MAX_MOTOR_TORQUE_NM;
    cmd.feedforward_torque = m_lastCommandedTorqueNm;

    // Using Begin rather than Set to avoid blocking calls.
    m_moteus->BeginPosition(cmd, &kPositionFormat);
      
    if (kDebug) {
      if (Serial) {
        auto isEqualWithinTolerance = [](float a, float b, float tolerance) {
          return fabs(a - b) <= tolerance;
        };

        if (!isEqualWithinTolerance(m_targetTorqueNm, m_lastCommandedTorqueNm, Constants::ACCELERATION_TOLERANCE_MSS)) {
          // Check to see whether the latest target is different within the controllers precision
          Serial.printf("[RW] [DEBUG] ReactionWheel::service() - Sent new command: target torque = %.4f Nm\r\n",
                        m_lastCommandedTorqueNm);
        }
        // Once a second, print the last command
        static uint32_t lastDebugTime = 0;
        if (millis() - lastDebugTime >= 1000) {
          lastDebugTime = millis();
          Serial.printf("[RW] [DEBUG] ReactionWheel::service() - Last command: target torque = %.4f Nm\r\n",
                        m_lastCommandedTorqueNm);
        }
      }
    }

    // Poll for the result of the last command and update the status
    if (m_moteus->Poll()) {
      m_lastMessageTime = millis();
      const Moteus::Query::Result v = m_moteus->last_result().values;
      parseMoteusStatus(m_status, v);
    }
  }
}

void ReactionWheel::parseMoteusStatus(RWStatus& status, const Moteus::Query::Result& v) {
  status.angularVelocityHz = v.velocity;
  status.angularMomentumKGM2S = Constants::WHEEL_INERTIA * v.velocity * Constants::HZ_TO_RAD_S;
  status.motorControllerMode = static_cast<uint8_t>(v.mode);
  status.motorControllerFaultCode = v.fault;
  switch (v.mode) {
    case mjbots::moteus::Mode::kFault:
      status.rwMode = RWMode::kFault;
      status.rwFault = RWFault::kMoteusFault;
      break;
    case mjbots::moteus::Mode::kPositionTimeout:
      status.rwMode = RWMode::kFault;
      status.rwFault = RWFault::kCANTimeout;
      break;
    default:
      status.rwMode = RWMode::kRunning;
      status.rwFault = RWFault::kNoFault;
      break;
  }
}

