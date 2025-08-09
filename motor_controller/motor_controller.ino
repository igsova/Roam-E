// Motor A connections
const int STBY = 9;
const int PWMA = 3;
const int AIN1 = 5;
const int AIN2 = 4;

// Motor B connections
const int PWMB = 6;
const int BIN1 = 7;
const int BIN2 = 8;

// Encoder A connections
const int ENCODER_A_A = 2; // Interrupt Pin
const int ENCODER_A_B = 11;

// Encoder B connections
const int ENCODER_B_A = 10;
const int ENCODER_B_B = 12;

// Volatile variables for encoder counts, modified by ISRs
volatile long encoderA_pos = 0;
volatile long encoderB_pos = 0;

void setup() {
  Serial.begin(115200);

  // Motor Driver Pin Modes
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Enable the motor driver
  digitalWrite(STBY, HIGH);

  // Encoder Pin Modes
  pinMode(ENCODER_A_A, INPUT_PULLUP);
  pinMode(ENCODER_A_B, INPUT_PULLUP);
  pinMode(ENCODER_B_A, INPUT_PULLUP);
  pinMode(ENCODER_B_B, INPUT_PULLUP);

  // Attach interrupts for encoders
  // Using interrupts is crucial for not missing encoder pulses.
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_A), readEncoderA, RISING);
  // Note: Pins 10 and 12 are not external interrupt pins on the Nano.
  // We would need to use Pin Change Interrupts for a more robust solution on these pins.
  // For simplicity, we'll poll them in the loop, but be aware this might miss fast pulses.
  // A better wiring would use D3 for the second encoder's interrupt pin.
}

void loop() {
  // Check for serial commands from Raspberry Pi
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    parseCommand(command);
  }

  // Since Encoder B is not on an interrupt pin, we poll it.
  // This is not ideal. For better performance, wire Encoder B's signal A to D3.
  static int lastEncoderB_A_State = LOW;
  int newEncoderB_A_State = digitalRead(ENCODER_B_A);
  if (newEncoderB_A_State != lastEncoderB_A_State && newEncoderB_A_State == HIGH) {
      readEncoderB();
  }
  lastEncoderB_A_State = newEncoderB_A_State;
}

void parseCommand(String cmd) {
  if (cmd == "GET_ENCODERS") {
    Serial.print(encoderA_pos);
    Serial.print(",");
    Serial.println(encoderB_pos);
  } else if (cmd.startsWith("SET_MOTOR")) {
    // Command format: SET_MOTOR,<A or B>,<speed>,<direction>
    // speed: 0-255, direction: 1=forward, -1=backward, 0=stop
    int firstComma = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);
    int thirdComma = cmd.indexOf(',', secondComma + 1);

    char motor = cmd.substring(firstComma + 1, secondComma).charAt(0);
    int speed = cmd.substring(secondComma + 1, thirdComma).toInt();
    int dir = cmd.substring(thirdComma + 1).toInt();

    if (motor == 'A') {
      setMotorSpeed(PWMA, AIN1, AIN2, speed, dir);
    } else if (motor == 'B') {
      setMotorSpeed(PWMB, BIN1, BIN2, speed, dir);
    }
  }
}

void setMotorSpeed(int pwmPin, int in1Pin, int in2Pin, int speed, int direction) {
  // Constrain speed to 0-255
  speed = constrain(speed, 0, 255);

  if (direction == 1) { // Forward
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (direction == -1) { // Backward
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else { // Stop (Brake)
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
    speed = 0;
  }
  analogWrite(pwmPin, speed);
}

// Interrupt Service Routine for Encoder A
void readEncoderA() {
  if (digitalRead(ENCODER_A_B) == LOW) {
    encoderA_pos++; // Clockwise
  } else {
    encoderA_pos--; // Counter-clockwise
  }
}

// Function to read Encoder B (called from loop)
void readEncoderB() {
    if (digitalRead(ENCODER_B_B) == LOW) {
        encoderB_pos++; // Clockwise
    } else {
        encoderB_pos--; // Counter-clockwise
    }
}

