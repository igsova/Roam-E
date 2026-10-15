// Opt-in ID_V1 (from V3): bounded paired straight-motion startup and PI handoff,
// plus an identification-only fixed-duty RUN mode (DUTY) behind the same guards.
// Equal same-sign targets only; six-second deadline and per-wheel1400-tick budget.
// No voltage/current/cell sensing. Default disabled; not production calibration.
#pragma once
#include <stdint.h>

const int BOUNDED_START_PWM = 128;
const int BOUNDED_START_PWM_STEP = 5;
const uint32_t BOUNDED_START_TIMEOUT_MS = 300;
const uint32_t BOUNDED_PROGRESS_TIMEOUT_MS = 250;
const uint32_t BOUNDED_EPISODE_TIMEOUT_MS = 6000;
const long BOUNDED_MAX_TRAVEL_TICKS = 1400;
const long BOUNDED_MAX_PAIR_SEPARATION_TICKS = 160;
const uint32_t BOUNDED_CONTROL_GAP_MS = 150;
const float BOUNDED_MAX_SPEED = 1000.0f;
const long BOUNDED_MIN_TARGET = 100;
const long BOUNDED_MAX_TARGET = 300;
// Identification: fixed duty replaces PI only after independent RUN proof.
// Startup boost/proof is unchanged and proves against this nominal speed.
const int ID_MIN_DUTY = 40;
const int ID_MAX_DUTY = 120;
const long ID_PROOF_TICKS_PER_SECOND = 200;

enum BoundedPhase : uint8_t { BOUNDED_IDLE, BOUNDED_STARTING, BOUNDED_RUNNING };
struct BoundedWheelState {
  BoundedPhase phase;
  uint32_t startedMs;
  uint32_t lastProgressMs;
  int32_t startTicks;
  int8_t direction;
  uint8_t proofFrames;
  bool responded;
  int startupPwm;
};
bool boundedStartupEnabled = false;
bool boundedFaultLatched = false;
uint8_t boundedFaultCode = 0;
BoundedWheelState boundedLeft = {};
BoundedWheelState boundedRight = {};
bool idDutyMode = false;
int idLeftDuty = 0;
int idRightDuty = 0;

uint32_t boundedElapsed(uint32_t now, uint32_t then) { return now - then; }
int32_t boundedCounterDelta(long current, long previous) {
  return (int32_t)((uint32_t)current - (uint32_t)previous);
}
float boundedAbs(float value) { return value < 0 ? -value : value; }
float boundedClamp(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}
void resetBoundedWheels() {
  boundedLeft = {};
  boundedRight = {};
}
void beginBoundedWheel(BoundedWheelState &state, long target, long previousTarget,
                       uint32_t now, long ticks) {
  if (target == 0) {
    state = {};
  } else if (previousTarget == 0) {
    state = {};
    state.phase = BOUNDED_STARTING;
    state.startedMs = state.lastProgressMs = now;
    state.startTicks = (int32_t)ticks;
    state.direction = target > 0 ? 1 : -1;
    state.startupPwm = BOUNDED_START_PWM;
  }
  // Same-direction command refresh/target change never restarts the boost.
}
bool boundedFail(uint8_t reason) {
  boundedFaultCode = reason;
  return false;
}
bool boundedWheelStep(BoundedWheelState &state, long target, long ticks,
                      long tickDelta, float measured, float feedForward,
                      float error, float dt, uint32_t now, float &integral,
                      int &output, int fixedDuty = 0) {
  if (target == 0) {
    integral = 0;
    output = 0;
    state = {};
    return true; // Unpowered wheel may roll passively on the floor.
  }
  if (state.phase == BOUNDED_IDLE) return boundedFail(1);
  if (boundedElapsed(now, state.startedMs) >= BOUNDED_EPISODE_TIMEOUT_MS)
    return boundedFail(2);
  if (boundedAbs(tickDelta / dt) > BOUNDED_MAX_SPEED ||
      boundedAbs(measured) > BOUNDED_MAX_SPEED) return boundedFail(3);
  const float totalProgress = state.direction * (float)boundedCounterDelta(ticks, state.startTicks);
  if (totalProgress > BOUNDED_MAX_TRAVEL_TICKS) return boundedFail(8);
  const float progress = state.direction * (float)tickDelta;
  if (progress < 0) return boundedFail(4);
  if (progress > 0) {
    state.lastProgressMs = now;
    state.responded = true;
  }
  if (state.responded && boundedElapsed(now, state.lastProgressMs) >= BOUNDED_PROGRESS_TIMEOUT_MS)
    return boundedFail(5);

  if (state.phase == BOUNDED_STARTING) {
    const uint32_t elapsed = boundedElapsed(now, state.startedMs);
    if (elapsed >= BOUNDED_START_TIMEOUT_MS) return boundedFail(6);
    integral = 0; // Never wind up the PI integrator while starting/stalled.
    const uint32_t step = elapsed / VELOCITY_CONTROL_INTERVAL_MS;
    const uint32_t increments = step > 0 ? step - 1 : 0;
    const int requested = BOUNDED_START_PWM + (int)increments * BOUNDED_START_PWM_STEP;
    state.startupPwm = requested > MAX_SAFE_PWM ? MAX_SAFE_PWM : requested;
    const float minimumSpeed = boundedAbs((float)target) * 0.5f;
    if (totalProgress >= 8 && state.direction * measured >= minimumSpeed) {
      if (state.proofFrames < 2) state.proofFrames++;
    } else {
      state.proofFrames = 0;
    }
    if (state.proofFrames < 2) {
      output = state.direction * state.startupPwm;
      return true;
    }
    state.phase = BOUNDED_RUNNING;
    // Starting friction is not a steady-running feed-forward requirement.
    // Do not seed the integrator with the breakaway boost: drop to the configured
    // feed-forward + proportional feedback after independent RUN proof.
    // Zero remains within the unchanged +/-45 anti-windup bound, both signs.
    integral = 0;
  } else {
    const float proposedIntegral = boundedClamp(integral + velocityKi * error * dt,
                                               -INTEGRAL_PWM_LIMIT, INTEGRAL_PWM_LIMIT);
    const float proposed = feedForward + velocityKp * error + proposedIntegral;
    const float low = state.direction > 0 ? 0.0f : -(float)MAX_SAFE_PWM;
    const float high = state.direction > 0 ? (float)MAX_SAFE_PWM : 0.0f;
    const float clipped = boundedClamp(proposed, low, high);
    // Conditional anti-windup for ceiling and commanded-direction zero bound.
    if (error * (proposed - clipped) <= 0) integral = proposedIntegral;
  }
  if (fixedDuty != 0) {
    // Identification RUN: open-loop duty in the proven direction only.
    if ((fixedDuty > 0) != (state.direction > 0)) return boundedFail(10);
    integral = 0;
    output = fixedDuty;
    return true;
  }
  const float low = state.direction > 0 ? 0.0f : -(float)MAX_SAFE_PWM;
  const float high = state.direction > 0 ? (float)MAX_SAFE_PWM : 0.0f;
  const float result = boundedClamp(feedForward + velocityKp * error + integral, low, high);
  output = result >= 0 ? (int)(result + 0.5f) : (int)(result - 0.5f);
  return true;
}
