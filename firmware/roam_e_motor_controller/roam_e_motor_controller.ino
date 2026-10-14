// Production image from 2026-10-14 (promoted from candidate r19_dual_p230):
// 90% running ceiling (230/255), opt-in bounded startup (START,1; default off)
// with its boost capped at 153, and velocity defaults measured on the R19 body.
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

// Two DRV8871 carriers, one per motor. Both inputs are PWM-capable so
// forward and reverse use the same drive/coast modulation strategy.
const uint8_t LEFT_IN1 = 10;
const uint8_t LEFT_IN2 = 5;

const uint8_t RIGHT_IN1 = 9;
const uint8_t RIGHT_IN2 = 6;

// Logical PWM/velocity is camera-forward positive. On the R19 assembly,
// positive right-channel PWM was visually confirmed backward (2026-10-11).
// Invert only the electrical bridge output; encoder normalization and
// reported PWM/velocity signs remain in the logical wheel convention.
const int8_t LEFT_MOTOR_POLARITY = 1;
const int8_t RIGHT_MOTOR_POLARITY = -1;

// Encoder pins
const uint8_t ENCODER_A_A = 2;
const uint8_t ENCODER_A_B = 11;
const uint8_t ENCODER_B_A = 3;
const uint8_t ENCODER_B_B = 12;

// Measured using CHANGE interrupts on encoder channel A (x2 decoding).
const long ENCODER_TICKS_PER_REVOLUTION = 748;

// 90% duty ceiling (230/255) since 2026-10-14, replacing 60%.
// This ceiling is not a motor-current limit or a hardware safety rating.
const int MAX_SAFE_PWM = 230;
const long MAX_TARGET_TICKS_PER_SECOND = 2000;

// A moving command must be refreshed before this expires.
const unsigned long MOTOR_WATCHDOG_MS = 350;

// Closed-loop velocity control at 50 Hz; telemetry at 20 Hz.
const unsigned long VELOCITY_CONTROL_INTERVAL_MS = 20;
const unsigned long TELEMETRY_INTERVAL_MS = 50;

// Floor-tested DRV8871 starting values. They can be changed at runtime with:
// PID,<kp_milli>,<ki_milli>,<kv_milli>,<static_pwm>
// FF,<left_forward>,<left_reverse>,<right_forward>,<right_reverse>
// R19 defaults (2026-10-14): PI 0.20/0.80 with Kv 0.09 PWM per tick/s and
// per-wheel/direction static PWM from 13-14 Oct floor holds and speed steps.
float velocityKp = 0.200f;
float velocityKi = 0.800f;
float velocityKv = 0.090f;
int leftForwardStaticPwm = 70;
int leftReverseStaticPwm = 55;
int rightForwardStaticPwm = 42;
int rightReverseStaticPwm = 63;

const float VELOCITY_FILTER_ALPHA = 0.35f;
const float INTEGRAL_PWM_LIMIT = 45.0f;

enum ControlMode : uint8_t {
  CONTROL_STOPPED = 0,
  CONTROL_DIRECT_PWM = 1,
  CONTROL_VELOCITY = 2,
};

volatile long encoderA_raw = 0;
volatile long encoderB_raw = 0;

ControlMode controlMode = CONTROL_STOPPED;
int currentLeftPwm = 0;
int currentRightPwm = 0;
long targetLeftTicksPerSecond = 0;
long targetRightTicksPerSecond = 0;
float measuredLeftTicksPerSecond = 0.0f;
float measuredRightTicksPerSecond = 0.0f;
float leftIntegralPwm = 0.0f;
float rightIntegralPwm = 0.0f;
long previousControlLeftTicks = 0;
long previousControlRightTicks = 0;

bool motorsActive = false;
unsigned long lastMotorCommandMs = 0;
unsigned long lastVelocityControlMs = 0;
unsigned long lastTelemetryMs = 0;

char receiveBuffer[64];
uint8_t receiveLength = 0;
bool discardingLongLine = false;

#include "bounded_startup.h"

void encoderAInterrupt() {
  if (digitalRead(ENCODER_A_A) == digitalRead(ENCODER_A_B)) {
    encoderA_raw++;
  } else {
    encoderA_raw--;
  }
}

void encoderBInterrupt() {
  if (digitalRead(ENCODER_B_A) == digitalRead(ENCODER_B_B)) {
    encoderB_raw++;
  } else {
    encoderB_raw--;
  }
}

void readNormalizedEncoders(long &left, long &right) {
  long rawA;
  long rawB;

  noInterrupts();
  rawA = encoderA_raw;
  rawB = encoderB_raw;
  interrupts();

  // Forward motion is positive for both wheels.
  left = rawA;
  right = -rawB;
}

void removeMotorPower() {
  analogWrite(LEFT_IN1, 0);
  analogWrite(LEFT_IN2, 0);
  analogWrite(RIGHT_IN1, 0);
  analogWrite(RIGHT_IN2, 0);

  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);

  currentLeftPwm = 0;
  currentRightPwm = 0;
  motorsActive = false;
}

void resetVelocityState() {
  targetLeftTicksPerSecond = 0;
  targetRightTicksPerSecond = 0;
  measuredLeftTicksPerSecond = 0.0f;
  measuredRightTicksPerSecond = 0.0f;
  leftIntegralPwm = 0.0f;
  rightIntegralPwm = 0.0f;
  resetBoundedWheels();
}

void stopControl() {
  controlMode = CONTROL_STOPPED;
  resetVelocityState();
  removeMotorPower();
}

void setDrv8871Output(uint8_t in1, uint8_t in2, int pwm) {
  if (pwm > 0) {
    // Preserve the previous bridge polarity: IN1 low, PWM on IN2.
    analogWrite(in1, 0);
    analogWrite(in2, pwm);
  } else if (pwm < 0) {
    analogWrite(in2, 0);
    analogWrite(in1, -pwm);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void applyMotorOutput(int leftPwm, int rightPwm) {
  leftPwm = constrain(leftPwm, -MAX_SAFE_PWM, MAX_SAFE_PWM);
  rightPwm = constrain(rightPwm, -MAX_SAFE_PWM, MAX_SAFE_PWM);

  if (leftPwm == 0 && rightPwm == 0) {
    removeMotorPower();
    return;
  }

  const bool leftReversing =
    currentLeftPwm != 0 &&
    leftPwm != 0 &&
    ((currentLeftPwm > 0) != (leftPwm > 0));

  const bool rightReversing =
    currentRightPwm != 0 &&
    rightPwm != 0 &&
    ((currentRightPwm > 0) != (rightPwm > 0));

  // Remove power briefly before reversing either motor.
  if (leftReversing || rightReversing) {
    analogWrite(LEFT_IN1, 0);
    analogWrite(LEFT_IN2, 0);
    analogWrite(RIGHT_IN1, 0);
    analogWrite(RIGHT_IN2, 0);
    delayMicroseconds(100);
  }

  setDrv8871Output(LEFT_IN1, LEFT_IN2, LEFT_MOTOR_POLARITY * leftPwm);
  setDrv8871Output(RIGHT_IN1, RIGHT_IN2, RIGHT_MOTOR_POLARITY * rightPwm);

  currentLeftPwm = leftPwm;
  currentRightPwm = rightPwm;
  motorsActive = true;
}

bool parsePairCommand(
  const char *command,
  const char *prefix,
  long &left,
  long &right
) {
  const size_t prefixLength = strlen(prefix);

  if (strncmp(command, prefix, prefixLength) != 0) {
    return false;
  }

  char *end;
  const char *cursor = command + prefixLength;
  const long parsedLeft = strtol(cursor, &end, 10);

  if (end == cursor || *end != ',') {
    return false;
  }

  cursor = end + 1;
  const long parsedRight = strtol(cursor, &end, 10);

  if (end == cursor || *end != '\0') {
    return false;
  }

  left = parsedLeft;
  right = parsedRight;
  return true;
}

bool parseFourValueCommand(
  const char *command,
  const char *prefix,
  long &first,
  long &second,
  long &third,
  long &fourth
) {
  const size_t prefixLength = strlen(prefix);
  if (strncmp(command, prefix, prefixLength) != 0) {
    return false;
  }

  long *values[] = {&first, &second, &third, &fourth};
  const char *cursor = command + prefixLength;
  char *end;

  for (uint8_t index = 0; index < 4; index++) {
    *values[index] = strtol(cursor, &end, 10);

    if (end == cursor) {
      return false;
    }

    const char expected = index < 3 ? ',' : '\0';
    if (*end != expected) {
      return false;
    }

    cursor = end + 1;
  }

  return true;
}

void beginVelocityControl(long leftTarget, long rightTarget) {
  if (leftTarget == 0 && rightTarget == 0) {
    stopControl();
    return;
  }

  if (controlMode != CONTROL_VELOCITY) {
    removeMotorPower();
    resetVelocityState();
    readNormalizedEncoders(
      previousControlLeftTicks,
      previousControlRightTicks
    );
    lastVelocityControlMs = millis();
    controlMode = CONTROL_VELOCITY;
  }

  if (
    targetLeftTicksPerSecond == 0 ||
    (targetLeftTicksPerSecond > 0) != (leftTarget > 0)
  ) {
    leftIntegralPwm = 0.0f;
  }

  if (
    targetRightTicksPerSecond == 0 ||
    (targetRightTicksPerSecond > 0) != (rightTarget > 0)
  ) {
    rightIntegralPwm = 0.0f;
  }

  if (boundedStartupEnabled) {
    long leftTicks, rightTicks;
    readNormalizedEncoders(leftTicks, rightTicks);
    const uint32_t now = (uint32_t)millis();
    beginBoundedWheel(boundedLeft, leftTarget, targetLeftTicksPerSecond, now, leftTicks);
    beginBoundedWheel(boundedRight, rightTarget, targetRightTicksPerSecond, now, rightTicks);
  }
  targetLeftTicksPerSecond = leftTarget;
  targetRightTicksPerSecond = rightTarget;
  lastMotorCommandMs = millis();
}

float feedForwardPwm(
  long target,
  int forwardStaticPwm,
  int reverseStaticPwm
) {
  if (target == 0) {
    return 0.0f;
  }

  const float staticPwm =
    target > 0 ? forwardStaticPwm : -reverseStaticPwm;
  return staticPwm + velocityKv * target;
}

int roundedPwm(float value) {
  if (value >= 0.0f) {
    return (int)(value + 0.5f);
  }

  return (int)(value - 0.5f);
}

long roundedTicksPerSecond(float value) {
  if (value >= 0.0f) {
    return (long)(value + 0.5f);
  }

  return (long)(value - 0.5f);
}

long signedCounterDelta(long current, long previous) {
  // Subtract in the unsigned domain so the 32-bit encoder counter can wrap
  // without invoking signed-overflow behaviour.
  return (long)((unsigned long)current - (unsigned long)previous);
}

void updateVelocityControl() {
  if (controlMode != CONTROL_VELOCITY) {
    return;
  }

  const unsigned long now = millis();
  const unsigned long elapsedMs = now - lastVelocityControlMs;

  if (elapsedMs < VELOCITY_CONTROL_INTERVAL_MS) {
    return;
  }

  if (boundedStartupEnabled && elapsedMs > BOUNDED_CONTROL_GAP_MS) {
    stopControl();
    boundedFaultLatched = true;
    Serial.println(F("EVENT,BOUNDED_FAULT,7"));
    return;
  }
  lastVelocityControlMs = now;

  long leftTicks;
  long rightTicks;
  readNormalizedEncoders(leftTicks, rightTicks);

  const long deltaLeft = boundedStartupEnabled ?
    boundedCounterDelta(leftTicks, previousControlLeftTicks) :
    signedCounterDelta(leftTicks, previousControlLeftTicks);
  const long deltaRight = boundedStartupEnabled ?
    boundedCounterDelta(rightTicks, previousControlRightTicks) :
    signedCounterDelta(rightTicks, previousControlRightTicks);
  previousControlLeftTicks = leftTicks;
  previousControlRightTicks = rightTicks;

  const float dt = elapsedMs / 1000.0f;
  const float rawLeftVelocity = deltaLeft / dt;
  const float rawRightVelocity = deltaRight / dt;

  measuredLeftTicksPerSecond =
    VELOCITY_FILTER_ALPHA * rawLeftVelocity +
    (1.0f - VELOCITY_FILTER_ALPHA) * measuredLeftTicksPerSecond;
  measuredRightTicksPerSecond =
    VELOCITY_FILTER_ALPHA * rawRightVelocity +
    (1.0f - VELOCITY_FILTER_ALPHA) * measuredRightTicksPerSecond;

  const float leftError =
    targetLeftTicksPerSecond - measuredLeftTicksPerSecond;
  const float rightError =
    targetRightTicksPerSecond - measuredRightTicksPerSecond;

  if (boundedStartupEnabled) {
    int leftOutput = 0, rightOutput = 0;
    const bool leftGood = boundedWheelStep(
      boundedLeft, targetLeftTicksPerSecond, leftTicks, deltaLeft,
      measuredLeftTicksPerSecond,
      feedForwardPwm(targetLeftTicksPerSecond, leftForwardStaticPwm, leftReverseStaticPwm),
      leftError, dt, (uint32_t)now, leftIntegralPwm, leftOutput);
    bool rightGood = leftGood && boundedWheelStep(
      boundedRight, targetRightTicksPerSecond, rightTicks, deltaRight,
      measuredRightTicksPerSecond,
      feedForwardPwm(targetRightTicksPerSecond, rightForwardStaticPwm, rightReverseStaticPwm),
      rightError, dt, (uint32_t)now, rightIntegralPwm, rightOutput);
    if (leftGood && rightGood) {
      const float leftProgress = boundedLeft.direction *
        (float)boundedCounterDelta(leftTicks,boundedLeft.startTicks);
      const float rightProgress = boundedRight.direction *
        (float)boundedCounterDelta(rightTicks,boundedRight.startTicks);
      if (boundedAbs(leftProgress-rightProgress) > BOUNDED_MAX_PAIR_SEPARATION_TICKS)
        rightGood = boundedFail(9);
    }
    if (!leftGood || !rightGood) {
      const uint8_t reason = boundedFaultCode;
      stopControl();
      boundedFaultLatched = true;
      Serial.print(F("EVENT,BOUNDED_FAULT,"));
      Serial.println((int)reason);
      return;
    }
    applyMotorOutput(leftOutput, rightOutput);
    return;
  }

  if (targetLeftTicksPerSecond == 0) {
    leftIntegralPwm = 0.0f;
  } else {
    leftIntegralPwm = constrain(
      leftIntegralPwm + velocityKi * leftError * dt,
      -INTEGRAL_PWM_LIMIT,
      INTEGRAL_PWM_LIMIT
    );
  }

  if (targetRightTicksPerSecond == 0) {
    rightIntegralPwm = 0.0f;
  } else {
    rightIntegralPwm = constrain(
      rightIntegralPwm + velocityKi * rightError * dt,
      -INTEGRAL_PWM_LIMIT,
      INTEGRAL_PWM_LIMIT
    );
  }

  const float leftOutput =
    feedForwardPwm(
      targetLeftTicksPerSecond,
      leftForwardStaticPwm,
      leftReverseStaticPwm
    ) +
    velocityKp * leftError +
    leftIntegralPwm;
  const float rightOutput =
    feedForwardPwm(
      targetRightTicksPerSecond,
      rightForwardStaticPwm,
      rightReverseStaticPwm
    ) +
    velocityKp * rightError +
    rightIntegralPwm;

  applyMotorOutput(
    targetLeftTicksPerSecond == 0 ? 0 : roundedPwm(leftOutput),
    targetRightTicksPerSecond == 0 ? 0 : roundedPwm(rightOutput)
  );
}

void sendEncoderReading() {
  long left;
  long right;
  readNormalizedEncoders(left, right);

  Serial.print(F("ENC,"));
  Serial.print(left);
  Serial.print(',');
  Serial.println(right);
}

void sendControllerInfo() {
  Serial.print(F("INFO,2,"));
  Serial.print(ENCODER_TICKS_PER_REVOLUTION);
  Serial.print(',');
  Serial.print(MAX_SAFE_PWM);
  Serial.print(',');
  Serial.println(MAX_TARGET_TICKS_PER_SECOND);
}

void handleCommand(const char *command) {
  if (strcmp(command, "STOP") == 0) {
    stopControl();
    boundedFaultLatched = false;
    boundedFaultCode = 0;
    Serial.println(F("OK,STOP"));
    return;
  }

  if (strcmp(command, "PING") == 0) {
    Serial.println(F("PONG"));
    return;
  }

  if (strcmp(command, "INFO") == 0) {
    sendControllerInfo();
    return;
  }

  if (strcmp(command, "ENC") == 0) {
    sendEncoderReading();
    return;
  }

  if (strcmp(command, "ZERO") == 0) {
    if (controlMode != CONTROL_STOPPED || motorsActive) {
      Serial.println(F("ERR,MOTORS_ACTIVE"));
      return;
    }

    noInterrupts();
    encoderA_raw = 0;
    encoderB_raw = 0;
    interrupts();

    Serial.println(F("OK,ZERO"));
    return;
  }

  if (strcmp(command, "CAPS") == 0) {
    Serial.println(F("CAPS,BOUNDED_DUAL_V3_P230B3,128,153,230,300,250,1000,6000,1400,160,3"));
    return;
  }
  if (strcmp(command, "START,1") == 0 || strcmp(command, "START,0") == 0) {
    if (controlMode != CONTROL_STOPPED || motorsActive) {
      Serial.println(F("ERR,MOTORS_ACTIVE"));
      return;
    }
    if (boundedFaultLatched) {
      Serial.println(F("ERR,CONTROL_FAULT"));
      return;
    }
    boundedStartupEnabled = strcmp(command, "START,1") == 0;
    resetBoundedWheels();
    Serial.println(boundedStartupEnabled ? F("OK,START,1") : F("OK,START,0"));
    return;
  }

  long leftValue;
  long rightValue;

  if (parsePairCommand(command, "PWM,", leftValue, rightValue)) {
    if (boundedStartupEnabled && (leftValue != 0 || rightValue != 0)) {
      stopControl();
      boundedFaultLatched = true;
      Serial.println(F("ERR,CAL_VEL_ONLY"));
      return;
    }
    if (
      leftValue < -MAX_SAFE_PWM ||
      leftValue > MAX_SAFE_PWM ||
      rightValue < -MAX_SAFE_PWM ||
      rightValue > MAX_SAFE_PWM
    ) {
      stopControl();
      Serial.println(F("ERR,RANGE"));
      return;
    }

    if (leftValue == 0 && rightValue == 0) {
      stopControl();
    } else {
      stopControl();
      controlMode = CONTROL_DIRECT_PWM;
      applyMotorOutput((int)leftValue, (int)rightValue);
      lastMotorCommandMs = millis();
    }

    Serial.print(F("OK,PWM,"));
    Serial.print(leftValue);
    Serial.print(',');
    Serial.println(rightValue);
    return;
  }

  if (parsePairCommand(command, "VEL,", leftValue, rightValue)) {
    if (
      leftValue < -MAX_TARGET_TICKS_PER_SECOND ||
      leftValue > MAX_TARGET_TICKS_PER_SECOND ||
      rightValue < -MAX_TARGET_TICKS_PER_SECOND ||
      rightValue > MAX_TARGET_TICKS_PER_SECOND
    ) {
      stopControl();
      Serial.println(F("ERR,RANGE"));
      return;
    }

    if (boundedFaultLatched) {
      Serial.println(F("ERR,CONTROL_FAULT"));
      return;
    }
    if (boundedStartupEnabled) {
      const long magnitude = leftValue != 0 ?
        (leftValue < 0 ? -leftValue : leftValue) :
        (rightValue < 0 ? -rightValue : rightValue);
      const bool reversal =
        (targetLeftTicksPerSecond != 0 && leftValue != 0 &&
         ((targetLeftTicksPerSecond > 0) != (leftValue > 0))) ||
        (targetRightTicksPerSecond != 0 && rightValue != 0 &&
         ((targetRightTicksPerSecond > 0) != (rightValue > 0)));
      // Opt-in V2 accepts only equal, same-direction paired targets (or all zero).
      if ((leftValue != rightValue) ||
          (magnitude != 0 && (magnitude < BOUNDED_MIN_TARGET || magnitude > BOUNDED_MAX_TARGET)) ||
          reversal) {
        stopControl();
        boundedFaultLatched = true;
        Serial.println(F("ERR,CAL_TARGET"));
        return;
      }
    }
    beginVelocityControl(leftValue, rightValue);
    Serial.print(F("OK,VEL,"));
    Serial.print(leftValue);
    Serial.print(',');
    Serial.println(rightValue);
    return;
  }

  long kpMilli;
  long kiMilli;
  long kvMilli;
  long staticPwm;

  if (
    parseFourValueCommand(
      command,
      "PID,",
      kpMilli,
      kiMilli,
      kvMilli,
      staticPwm
    )
  ) {
    if (controlMode != CONTROL_STOPPED || motorsActive) {
      Serial.println(F("ERR,MOTORS_ACTIVE"));
      return;
    }

    if (
      kpMilli < 0 || kpMilli > 1000 ||
      kiMilli < 0 || kiMilli > 2000 ||
      kvMilli < 0 || kvMilli > 1000 ||
      staticPwm < 0 || staticPwm > MAX_SAFE_PWM
    ) {
      Serial.println(F("ERR,RANGE"));
      return;
    }

    velocityKp = kpMilli / 1000.0f;
    velocityKi = kiMilli / 1000.0f;
    velocityKv = kvMilli / 1000.0f;
    leftForwardStaticPwm = (int)staticPwm;
    leftReverseStaticPwm = (int)staticPwm;
    rightForwardStaticPwm = (int)staticPwm;
    rightReverseStaticPwm = (int)staticPwm;

    Serial.print(F("OK,PID,"));
    Serial.print(kpMilli);
    Serial.print(',');
    Serial.print(kiMilli);
    Serial.print(',');
    Serial.print(kvMilli);
    Serial.print(',');
    Serial.println(staticPwm);
    return;
  }

  long leftForward;
  long leftReverse;
  long rightForward;
  long rightReverse;

  if (
    parseFourValueCommand(
      command,
      "FF,",
      leftForward,
      leftReverse,
      rightForward,
      rightReverse
    )
  ) {
    if (controlMode != CONTROL_STOPPED || motorsActive) {
      Serial.println(F("ERR,MOTORS_ACTIVE"));
      return;
    }

    if (
      leftForward < 0 || leftForward > MAX_SAFE_PWM ||
      leftReverse < 0 || leftReverse > MAX_SAFE_PWM ||
      rightForward < 0 || rightForward > MAX_SAFE_PWM ||
      rightReverse < 0 || rightReverse > MAX_SAFE_PWM
    ) {
      Serial.println(F("ERR,RANGE"));
      return;
    }

    leftForwardStaticPwm = (int)leftForward;
    leftReverseStaticPwm = (int)leftReverse;
    rightForwardStaticPwm = (int)rightForward;
    rightReverseStaticPwm = (int)rightReverse;

    Serial.print(F("OK,FF,"));
    Serial.print(leftForward);
    Serial.print(',');
    Serial.print(leftReverse);
    Serial.print(',');
    Serial.print(rightForward);
    Serial.print(',');
    Serial.println(rightReverse);
    return;
  }

  // Malformed or unknown motion input fails safe.
  stopControl();
  Serial.println(F("ERR,BAD_COMMAND"));
}

void processSerial() {
  while (Serial.available() > 0) {
    const char incoming = Serial.read();

    if (incoming == '\r') {
      continue;
    }

    if (discardingLongLine) {
      if (incoming == '\n') {
        discardingLongLine = false;
        receiveLength = 0;
        Serial.println(F("ERR,LINE_TOO_LONG"));
      }
      continue;
    }

    if (incoming == '\n') {
      receiveBuffer[receiveLength] = '\0';

      if (receiveLength > 0) {
        handleCommand(receiveBuffer);
      }

      receiveLength = 0;
      continue;
    }

    if (receiveLength < sizeof(receiveBuffer) - 1) {
      receiveBuffer[receiveLength++] = incoming;
    } else {
      stopControl();
      receiveLength = 0;
      discardingLongLine = true;
    }
  }
}

void publishTelemetry() {
  const unsigned long now = millis();

  if (now - lastTelemetryMs < TELEMETRY_INTERVAL_MS) {
    return;
  }

  lastTelemetryMs = now;

  long left;
  long right;
  readNormalizedEncoders(left, right);

  // The first six fields are protocol-v1 compatible. Protocol v2 appends
  // target and measured wheel velocities in normalized ticks per second.
  Serial.print(F("T,"));
  Serial.print(now);
  Serial.print(',');
  Serial.print(left);
  Serial.print(',');
  Serial.print(right);
  Serial.print(',');
  Serial.print(currentLeftPwm);
  Serial.print(',');
  Serial.print(currentRightPwm);
  Serial.print(',');
  Serial.print(targetLeftTicksPerSecond);
  Serial.print(',');
  Serial.print(targetRightTicksPerSecond);
  Serial.print(',');
  Serial.print(roundedTicksPerSecond(measuredLeftTicksPerSecond));
  Serial.print(',');
  Serial.println(roundedTicksPerSecond(measuredRightTicksPerSecond));
  if (boundedStartupEnabled) {
    Serial.print(F("B,"));
    Serial.print(now);
    Serial.print(',');
    Serial.print((int)boundedLeft.phase);
    Serial.print(',');
    Serial.print((int)boundedRight.phase);
    Serial.print(',');
    Serial.println(boundedFaultLatched ? 1 : 0);
  }
}

void setup() {
  // Preload all outputs LOW before enabling them.
  digitalWrite(LEFT_IN1, LOW);
  pinMode(LEFT_IN1, OUTPUT);
  digitalWrite(LEFT_IN2, LOW);
  pinMode(LEFT_IN2, OUTPUT);

  digitalWrite(RIGHT_IN1, LOW);
  pinMode(RIGHT_IN1, OUTPUT);
  digitalWrite(RIGHT_IN2, LOW);
  pinMode(RIGHT_IN2, OUTPUT);

  stopControl();

  pinMode(ENCODER_A_A, INPUT_PULLUP);
  pinMode(ENCODER_A_B, INPUT_PULLUP);
  pinMode(ENCODER_B_A, INPUT_PULLUP);
  pinMode(ENCODER_B_B, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_A_A),
    encoderAInterrupt,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_B_A),
    encoderBInterrupt,
    CHANGE
  );

  Serial.begin(115200);
  Serial.println(F("READY,ROAM_E_MOTOR_CONTROLLER,2"));
}

void loop() {
  processSerial();

  if (
    controlMode != CONTROL_STOPPED &&
    millis() - lastMotorCommandMs > MOTOR_WATCHDOG_MS
  ) {
    stopControl();
    Serial.println(F("EVENT,WATCHDOG_STOP"));
  }

  updateVelocityControl();

  // Reinforce the safe electrical state whenever idle.
  if (controlMode == CONTROL_STOPPED) {
    removeMotorPower();
  }

  publishTelemetry();
}
