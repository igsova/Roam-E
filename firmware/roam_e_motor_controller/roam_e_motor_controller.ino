#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

// Motor driver pins
const uint8_t PWMA = 10;
const uint8_t AIN1 = 5;
const uint8_t AIN2 = 4;

const uint8_t PWMB = 9;
const uint8_t BIN1 = 7;
const uint8_t BIN2 = 8;

// Encoder pins
const uint8_t ENCODER_A_A = 2;
const uint8_t ENCODER_A_B = 11;
const uint8_t ENCODER_B_A = 3;
const uint8_t ENCODER_B_B = 12;

// Measured using CHANGE interrupts on encoder channel A (x2 decoding).
const long ENCODER_TICKS_PER_REVOLUTION = 748;

// Temporary protection limits while tuning the drive base.
const int MAX_SAFE_PWM = 160;
const long MAX_TARGET_TICKS_PER_SECOND = 2000;

// A moving command must be refreshed before this expires.
const unsigned long MOTOR_WATCHDOG_MS = 350;

// Closed-loop velocity control at 50 Hz; telemetry at 20 Hz.
const unsigned long VELOCITY_CONTROL_INTERVAL_MS = 20;
const unsigned long TELEMETRY_INTERVAL_MS = 50;

// Conservative starting values. They can be changed at runtime with:
// PID,<kp_milli>,<ki_milli>,<kv_milli>,<static_pwm>
// FF,<left_forward>,<left_reverse>,<right_forward>,<right_reverse>
float velocityKp = 0.080f;
float velocityKi = 0.200f;
float velocityKv = 0.100f;
int leftForwardStaticPwm = 45;
int leftReverseStaticPwm = 36;
int rightForwardStaticPwm = 40;
int rightReverseStaticPwm = 39;

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
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

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
}

void stopControl() {
  controlMode = CONTROL_STOPPED;
  resetVelocityState();
  removeMotorPower();
}

void setLeftDirection(int pwm) {
  if (pwm > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else if (pwm < 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
}

void setRightDirection(int pwm) {
  // Right motor polarity is reversed by its physical mounting.
  if (pwm > 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else if (pwm < 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
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
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    delayMicroseconds(100);
  }

  setLeftDirection(leftPwm);
  setRightDirection(rightPwm);
  analogWrite(PWMA, abs(leftPwm));
  analogWrite(PWMB, abs(rightPwm));

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

  lastVelocityControlMs = now;

  long leftTicks;
  long rightTicks;
  readNormalizedEncoders(leftTicks, rightTicks);

  const long deltaLeft = signedCounterDelta(
    leftTicks,
    previousControlLeftTicks
  );
  const long deltaRight = signedCounterDelta(
    rightTicks,
    previousControlRightTicks
  );
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

  long leftValue;
  long rightValue;

  if (parsePairCommand(command, "PWM,", leftValue, rightValue)) {
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
}

void setup() {
  // Preload all outputs LOW before enabling them.
  digitalWrite(PWMA, LOW);
  pinMode(PWMA, OUTPUT);
  digitalWrite(AIN1, LOW);
  pinMode(AIN1, OUTPUT);
  digitalWrite(AIN2, LOW);
  pinMode(AIN2, OUTPUT);

  digitalWrite(PWMB, LOW);
  pinMode(PWMB, OUTPUT);
  digitalWrite(BIN1, LOW);
  pinMode(BIN1, OUTPUT);
  digitalWrite(BIN2, LOW);
  pinMode(BIN2, OUTPUT);

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
