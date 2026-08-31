#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

// Motor driver pins
// const uint8_t STBY = 9;

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

// Calibration
const long ENCODER_TICKS_PER_REVOLUTION = 748;

// Temporary protection limit while testing.
// Full Arduino PWM range is 0–255.
const int MAX_SAFE_PWM = 160;

// A moving command must be refreshed before this expires.
const unsigned long MOTOR_WATCHDOG_MS = 350;

// Encoder telemetry frequency: 10 Hz.
const unsigned long TELEMETRY_INTERVAL_MS = 100;

volatile long encoderA_raw = 0;
volatile long encoderB_raw = 0;

int currentLeftPwm = 0;
int currentRightPwm = 0;

bool motorsActive = false;
unsigned long lastMotorCommandMs = 0;
unsigned long lastTelemetryMs = 0;

char receiveBuffer[48];
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

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  // digitalWrite(STBY, LOW);

  currentLeftPwm = 0;
  currentRightPwm = 0;
  motorsActive = false;
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

void applyMotorCommand(int leftPwm, int rightPwm) {
  if (leftPwm == 0 && rightPwm == 0) {
    stopMotors();
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

  // digitalWrite(STBY, HIGH);

  analogWrite(PWMA, abs(leftPwm));
  analogWrite(PWMB, abs(rightPwm));

  currentLeftPwm = leftPwm;
  currentRightPwm = rightPwm;
  motorsActive = true;
}

bool parsePwmCommand(
  const char *command,
  int &leftPwm,
  int &rightPwm
) {
  if (strncmp(command, "PWM,", 4) != 0) {
    return false;
  }

  char *end;
  const char *cursor = command + 4;

  const long parsedLeft = strtol(cursor, &end, 10);

  if (end == cursor || *end != ',') {
    return false;
  }

  cursor = end + 1;
  const long parsedRight = strtol(cursor, &end, 10);

  if (end == cursor || *end != '\0') {
    return false;
  }

  if (
    parsedLeft < -MAX_SAFE_PWM ||
    parsedLeft > MAX_SAFE_PWM ||
    parsedRight < -MAX_SAFE_PWM ||
    parsedRight > MAX_SAFE_PWM
  ) {
    return false;
  }

  leftPwm = (int)parsedLeft;
  rightPwm = (int)parsedRight;
  return true;
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

void handleCommand(const char *command) {
  if (strcmp(command, "STOP") == 0) {
    stopMotors();
    Serial.println(F("OK,STOP"));
    return;
  }

  if (strcmp(command, "PING") == 0) {
    Serial.println(F("PONG"));
    return;
  }

  if (strcmp(command, "ENC") == 0) {
    sendEncoderReading();
    return;
  }

  if (strcmp(command, "ZERO") == 0) {
    if (motorsActive) {
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

  int leftPwm;
  int rightPwm;

  if (parsePwmCommand(command, leftPwm, rightPwm)) {
    applyMotorCommand(leftPwm, rightPwm);
    lastMotorCommandMs = millis();

    Serial.print(F("OK,PWM,"));
    Serial.print(leftPwm);
    Serial.print(',');
    Serial.println(rightPwm);
    return;
  }

  // Malformed or unknown motion input fails safe.
  stopMotors();
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
      stopMotors();
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

  Serial.print(F("T,"));
  Serial.print(now);
  Serial.print(',');
  Serial.print(left);
  Serial.print(',');
  Serial.print(right);
  Serial.print(',');
  Serial.print(currentLeftPwm);
  Serial.print(',');
  Serial.println(currentRightPwm);
}

void setup() {
  // Preload all outputs LOW before enabling them.
  // digitalWrite(STBY, LOW);
  // pinMode(STBY, OUTPUT);

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

  stopMotors();

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
  Serial.println(F("READY,ROAM_E_MOTOR_CONTROLLER,1"));
}

void loop() {
  processSerial();

  if (
    motorsActive &&
    millis() - lastMotorCommandMs > MOTOR_WATCHDOG_MS
  ) {
    stopMotors();
    Serial.println(F("EVENT,WATCHDOG_STOP"));
  }

  // Reinforce the safe state whenever idle.
  if (!motorsActive) {
    stopMotors();
  }

  publishTelemetry();
}
