#ifndef THRUSTER_CONTROL_H
#define THRUSTER_CONTROL_H
#include <Arduino.h>

// =======================================================================
// THRUSTER CONTROL — thrust command -> PWM duty cycle -> valve output
// =======================================================================
//
// MODEL
// thrust scales ~linearly with upstream pressure:
//
//      F_open(P) = K_F * P
//
// where F_open is the thrust produced while the valve is fully open, 
//
//      F_avg(D) = D * F_open(P)
//
// Inverting for control:
//
//      D_cmd = F_cmd / (K_F * P)      clamped to [D_MIN, 1.0]
//
// D_MIN is the shortest pulse width that still reliably fully opens/closes
// the valve 

namespace ThrusterControl {

// F is calculations are available in the report for reference.
//      K_F = F / P_gauge = 0.002410 N / 6.8 bar = 0.0003544 N/bar

//
// TODO: get this confirmed against the professor's review before flight
// use, per the "confirm all numbers with professor" step in the burn-time
// calc writeup.
constexpr float K_F_N_PER_BAR = 0.0003544f;

// Fixed pressure at the valve inlet — this is the REGULATED outlet
// pressure (PRV set point), not raw tank/canister pressure, since that's
// what the valve orifice actually sees. No sensor for now.
constexpr float TANK_PRESSURE_BAR_PLACEHOLDER = 6.8f;

// Minimum duty cycle the valve reliably tracks (dead-band). Supplier
// spec: 1 ms minimum pulse width. 
constexpr float D_MIN = 0.1f;

// Maximum duty cycle (safety headroom below 100% if desired). 1.0 = no cap.
constexpr float D_MAX = 1.0f;

// PWM carrier frequency, Hz — matches your driver circuit design.
constexpr uint16_t PWM_FREQUENCY_HZ = 100;

// Teensy 4.0 PWM resolution — 12-bit gives duty granularity of 1/4096.
constexpr uint8_t PWM_RESOLUTION_BITS = 12;
constexpr uint16_t PWM_MAX_COUNT = (1u << PWM_RESOLUTION_BITS) - 1;  // 4095

// What to do with a thrust command that maps below D_MIN.
enum class MinPulseBehaviour {
  kClampToMin,  // force D_MIN whenever F_cmd > 0 (valve fires slightly harder than requested)
  kCloseValve   // treat as zero thrust / valve fully closed (undershoots small commands)
};
constexpr MinPulseBehaviour MIN_PULSE_BEHAVIOUR = MinPulseBehaviour::kCloseValve;

// ---- Core conversion ----------------------------------------------------

// Returns the fully-open thrust available at the given pressure (N).
inline float openThrustAtPressure(float pressureBar) {
  return K_F_N_PER_BAR * pressureBar;
}

// Converts a commanded thrust (N) at a given pressure (bar) to a duty
// cycle in [0, D_MAX]. Returns 0.0f if F_cmd <= 0 or K_F is unset.
inline float thrustToDutyCycle(float thrustCmdN, float pressureBar) {
  if (thrustCmdN <= 0.0f) return 0.0f;

  const float fOpen = openThrustAtPressure(pressureBar);
  if (fOpen <= 0.0f) return 0.0f;  // avoid divide-by-zero if K_F unset

  float duty = thrustCmdN / fOpen;
  duty = constrain(duty, 0.0f, D_MAX);

  if (duty > 0.0f && duty < D_MIN) {
    duty = (MIN_PULSE_BEHAVIOUR == MinPulseBehaviour::kClampToMin) ? D_MIN : 0.0f;
  }
  return duty;
}

// Converts a duty cycle [0.0, 1.0] to a PWM count for analogWrite() at the
// configured resolution.
inline uint16_t dutyCycleToPwmCount(float duty) {
  duty = constrain(duty, 0.0f, 1.0f);
  return static_cast<uint16_t>(duty * PWM_MAX_COUNT + 0.5f);
}


inline void setupPin(uint8_t pin) {
  pinMode(pin, OUTPUT);
#if defined(TEENSYDUINO)
  analogWriteFrequency(pin, PWM_FREQUENCY_HZ);
  analogWriteResolution(PWM_RESOLUTION_BITS);
#else
  // analogWriteFrequency()/analogWriteResolution() are Teensy-core-only.
  // On other boards (e.g. Uno R4) this falls back to the board's default
 
#endif
}

inline float driveThruster(uint8_t pin, float thrustCmdN, float pressureBar) {
  const float duty = thrustToDutyCycle(thrustCmdN, pressureBar);
  analogWrite(pin, dutyCycleToPwmCount(duty));
  return duty;  // handy to stash in telemetry
}

// ---- Propellant tracking -------------------------------------------------

constexpr float MASS_FLOW_FULL_OPEN_G_PER_S = 0.005599f;

// Initial propellant mass in the 150 cm^3 storage tank at 120 bar, room
// temperature (293 K), via ideal gas law for N2: m = P*V/(R*T)
// = (120e5 Pa * 150e-6 m^3) / (296.8 J/kgK * 293 K) ~= 20.7 g
constexpr float INITIAL_TANK_MASS_G = 20.7f;

// Running total of propellant consumed since startup (grams). Resets to
// 0 on reboot -- not persisted across power cycles.
inline float g_consumedPropellantG = 0.0f;

// Updates the running consumption
// total and returns remaining propellant as a percentage of
// INITIAL_TANK_MASS_G, clamped to [0, 100].

inline float updatePropellantUsage(const float duties[4], float dtSeconds) {
  float totalDuty = 0.0f;
  for (uint8_t i = 0; i < 4; i++) totalDuty += duties[i];
  g_consumedPropellantG += totalDuty * MASS_FLOW_FULL_OPEN_G_PER_S * dtSeconds;

  float remainingG = INITIAL_TANK_MASS_G - g_consumedPropellantG;
  remainingG = constrain(remainingG, 0.0f, INITIAL_TANK_MASS_G);
  return (remainingG / INITIAL_TANK_MASS_G) * 100.0f;
}

}  // namespace ThrusterControl

#endif  // THRUSTER_CONTROL_H