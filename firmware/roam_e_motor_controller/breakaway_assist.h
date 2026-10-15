// Breakaway assist for normal (START,0) velocity control, added 2026-10-14.
// Nav2 asks for small in-place turns that the PI output (~70 PWM) cannot start
// from standstill, especially at low battery. When a commanded wheel shows no
// encoder progress, ramp a duty floor from 100 by +5 per control step, capped at
// the bounded-startup limit (153), until it moves. Give up after 500 ms, then
// wait 500 ms before another attempt so a jammed wheel is not driven
// continuously. The PI output is never reduced; only a floor is applied in the
// commanded direction. Opt-in bounded mode (START,1) is unaffected.
#pragma once
#include <stdint.h>

const int BREAKAWAY_START_PWM = 100;
const int BREAKAWAY_STEP_PWM = 5;
const int BREAKAWAY_MAX_PWM = 153;
const uint32_t BREAKAWAY_MIN_DETECT_MS = 80;
const uint32_t BREAKAWAY_MAX_DETECT_MS = 300;
const uint32_t BREAKAWAY_TIMEOUT_MS = 500;
const uint32_t BREAKAWAY_COOLDOWN_MS = 500;

struct BreakawayState {
  int8_t direction;
  uint32_t lastMoveMs;
  uint32_t boostStartMs;
  uint32_t cooldownUntilMs;
  bool boosting;
  bool coolingDown;
  int boostPwm;
};

BreakawayState breakawayLeft = {};
BreakawayState breakawayRight = {};

void resetBreakaway(BreakawayState &state) { state = {}; }

// Returns the duty floor magnitude (0 = none) for this control step.
int breakawayStep(BreakawayState &state, long target, long tickDelta, uint32_t now) {
  if (target == 0) {
    state = {};
    return 0;
  }
  const int8_t direction = target > 0 ? 1 : -1;
  if (state.direction != direction) {
    state = {};
    state.direction = direction;
    state.lastMoveMs = now;  // Detection window starts with the command.
  }
  if (direction * tickDelta > 0) {
    state.lastMoveMs = now;
    state.boosting = false;
    state.boostPwm = 0;
    return 0;
  }
  if (state.coolingDown) {
    if ((int32_t)(now - state.cooldownUntilMs) < 0) return 0;
    state.coolingDown = false;
    state.lastMoveMs = now;
    return 0;
  }
  const long magnitude = target > 0 ? target : -target;
  uint32_t detect = (uint32_t)(2500L / (magnitude > 0 ? magnitude : 1));
  if (detect < BREAKAWAY_MIN_DETECT_MS) detect = BREAKAWAY_MIN_DETECT_MS;
  if (detect > BREAKAWAY_MAX_DETECT_MS) detect = BREAKAWAY_MAX_DETECT_MS;
  if (now - state.lastMoveMs < detect) return 0;
  if (!state.boosting) {
    state.boosting = true;
    state.boostStartMs = now;
    state.boostPwm = BREAKAWAY_START_PWM;
  } else {
    state.boostPwm += BREAKAWAY_STEP_PWM;
    if (state.boostPwm > BREAKAWAY_MAX_PWM) state.boostPwm = BREAKAWAY_MAX_PWM;
  }
  if (now - state.boostStartMs >= BREAKAWAY_TIMEOUT_MS) {
    state.boosting = false;
    state.boostPwm = 0;
    state.coolingDown = true;
    state.cooldownUntilMs = now + BREAKAWAY_COOLDOWN_MS;
    return 0;
  }
  return state.boostPwm;
}

int applyBreakawayFloor(int output, long target, int floorPwm) {
  if (floorPwm <= 0 || target == 0) return output;
  if (target > 0) return output < floorPwm ? floorPwm : output;
  return output > -floorPwm ? -floorPwm : output;
}
