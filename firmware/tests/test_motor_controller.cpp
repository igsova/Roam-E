// Compiles and exercises the actual sketch against an entirely offline shim.
#include "Arduino.h"
#include <iostream>
#include <stdexcept>
#include <string>

#ifndef ROAM_MOTOR_SKETCH
// Default: the PWM153 production sketch as it was before the 14 Oct promotion.
// The promoted production sketch is covered by test_production_controller.cpp.
#define ROAM_MOTOR_SKETCH "../candidates/r19_pwm153_production_20261013/roam_e_motor_controller.ino"
#endif
#include ROAM_MOTOR_SKETCH

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

void outputs(int left1, int left2, int right1, int right2) {
  require(mockOutputs[LEFT_IN1] == left1, "Left IN1 mismatch");
  require(mockOutputs[LEFT_IN2] == left2, "Left IN2 mismatch");
  require(mockOutputs[RIGHT_IN1] == right1, "Right IN1 mismatch");
  require(mockOutputs[RIGHT_IN2] == right2, "Right IN2 mismatch");
}

void logical(int left, int right) {
  require(currentLeftPwm == left, "Logical left PWM changed");
  require(currentRightPwm == right, "Logical right PWM changed");
}

void test(const std::string &name) {
  setup();
  Serial.clearOutput();
  mockWrites.clear();

  if (name == "pins_and_limits") {
    require(LEFT_IN1 == 10 && LEFT_IN2 == 5 && RIGHT_IN1 == 9 && RIGHT_IN2 == 6,
            "Bridge pin assignment changed");
    require(ENCODER_A_A == 2 && ENCODER_A_B == 11 &&
            ENCODER_B_A == 3 && ENCODER_B_B == 12, "Encoder pins changed");
    require(MAX_SAFE_PWM == 153 && MAX_TARGET_TICKS_PER_SECOND == 2000 &&
            MOTOR_WATCHDOG_MS == 350, "Safety limits changed");
    require(LEFT_MOTOR_POLARITY == 1 && RIGHT_MOTOR_POLARITY == -1,
            "R19 polarity mismatch");
  } else if (name == "startup_stopped") {
    outputs(0, 0, 0, 0);
    logical(0, 0);
    require(!motorsActive && controlMode == CONTROL_STOPPED, "Startup not stopped");
  } else if (name == "left_positive") {
    handleCommand("PWM,100,0");
    outputs(0, 100, 0, 0);
    logical(100, 0);
  } else if (name == "right_positive") {
    handleCommand("PWM,0,100");
    outputs(0, 0, 100, 0);
    logical(0, 100);
  } else if (name == "left_negative") {
    handleCommand("PWM,-100,0");
    outputs(100, 0, 0, 0);
    logical(-100, 0);
  } else if (name == "right_negative") {
    handleCommand("PWM,0,-100");
    outputs(0, 0, 0, 100);
    logical(0, -100);
  } else if (name == "clamp_before_polarity") {
    applyMotorOutput(1000, -1000);
    outputs(0, 153, 0, 153);
    logical(153, -153);
  } else if (name == "zero_channel") {
    applyMotorOutput(80, 90);
    applyMotorOutput(0, 90);
    outputs(0, 0, 90, 0);
    logical(0, 90);
    applyMotorOutput(80, 0);
    outputs(0, 80, 0, 0);
    logical(80, 0);
  } else if (name == "stop") {
    handleCommand("VEL,100,100");
    mockMillis = 20;
    updateVelocityControl();
    handleCommand("STOP");
    outputs(0, 0, 0, 0);
    logical(0, 0);
    require(!motorsActive && controlMode == CONTROL_STOPPED &&
            targetLeftTicksPerSecond == 0 && targetRightTicksPerSecond == 0 &&
            leftIntegralPwm == 0 && rightIntegralPwm == 0,
            "STOP did not clear controller state");
    require(Serial.output.str().find("OK,STOP") != std::string::npos, "Missing STOP ack");
  } else if (name == "zero_command") {
    handleCommand("PWM,100,100");
    handleCommand("PWM,0,0");
    outputs(0, 0, 0, 0);
    require(controlMode == CONTROL_STOPPED && !motorsActive, "Zero PWM not stopped");
  } else if (name == "reversal_dead_time") {
    applyMotorOutput(80, 90);
    applyMotorOutput(80, -90);
    require(mockDelays.size() == 1 && mockDelays[0] == 100, "Reversal gap changed");
    const auto &atDelay = mockReversalSnapshots.back();
    require(atDelay[LEFT_IN1] == 0 && atDelay[LEFT_IN2] == 0 &&
            atDelay[RIGHT_IN1] == 0 && atDelay[RIGHT_IN2] == 0,
            "Outputs not zero during reversal delay");
    outputs(0, 80, 0, 90);
    logical(80, -90);
  } else if (name == "watchdog") {
    handleCommand("PWM,0,100");
    mockMillis = MOTOR_WATCHDOG_MS + 1;
    loop();
    outputs(0, 0, 0, 0);
    logical(0, 0);
    require(controlMode == CONTROL_STOPPED && !motorsActive, "Watchdog not stopped");
    require(Serial.output.str().find("EVENT,WATCHDOG_STOP") != std::string::npos,
            "Missing watchdog event");
  } else if (name == "watchdog_boundary") {
    handleCommand("PWM,0,100");
    mockMillis = MOTOR_WATCHDOG_MS;
    loop();
    outputs(0, 0, 100, 0);
    require(controlMode == CONTROL_DIRECT_PWM, "Watchdog boundary changed");
  } else if (name == "encoder_normalization") {
    encoderA_raw = 17;
    encoderB_raw = -23;
    long left, right;
    readNormalizedEncoders(left, right);
    require(left == 17 && right == 23, "Forward encoder normalization changed");
    encoderA_raw = -17;
    encoderB_raw = 23;
    readNormalizedEncoders(left, right);
    require(left == -17 && right == -23, "Reverse encoder normalization changed");
  } else if (name == "logical_telemetry") {
    handleCommand("PWM,0,100");
    Serial.clearOutput();
    mockMillis = 50;
    publishTelemetry();
    require(Serial.output.str() == "T,50,0,0,0,100,0,0,0,0\n",
            "Telemetry no longer reports logical PWM");
  } else if (name == "velocity_forward") {
    handleCommand("VEL,100,100");
    mockMillis = 20;
    updateVelocityControl();
    require(currentLeftPwm > 0 && currentRightPwm > 0, "Forward logical VEL sign changed");
    outputs(0, currentLeftPwm, currentRightPwm, 0);
    require(targetLeftTicksPerSecond == 100 && targetRightTicksPerSecond == 100,
            "Forward velocity targets changed");
  } else if (name == "velocity_reverse") {
    handleCommand("VEL,-100,-100");
    mockMillis = 20;
    updateVelocityControl();
    require(currentLeftPwm < 0 && currentRightPwm < 0, "Reverse logical VEL sign changed");
    outputs(-currentLeftPwm, 0, 0, -currentRightPwm);
    require(targetLeftTicksPerSecond == -100 && targetRightTicksPerSecond == -100,
            "Reverse velocity targets changed");
  } else if (name == "zero_velocity") {
    handleCommand("VEL,0,100");
    mockMillis = 20;
    updateVelocityControl();
    require(currentLeftPwm == 0 && currentRightPwm > 0, "Zero left VEL drove left wheel");
    outputs(0, 0, currentRightPwm, 0);
    handleCommand("VEL,0,0");
    outputs(0, 0, 0, 0);
    require(controlMode == CONTROL_STOPPED, "Zero VEL not stopped");
  } else if (name == "invalid_command") {
    handleCommand("PWM,0,100");
    handleCommand("invalid");
    outputs(0, 0, 0, 0);
    require(controlMode == CONTROL_STOPPED, "Invalid command not stopped");
  } else if (name == "range_rejection") {
    handleCommand("PWM,0,100");
    handleCommand("PWM,0,154");
    outputs(0, 0, 0, 0);
    require(Serial.output.str().find("ERR,RANGE") != std::string::npos,
            "PWM range check changed");
  } else if (name == "info_compatible") {
    handleCommand("INFO");
    require(Serial.output.str() == "INFO,2,748,153,2000\n", "INFO ceiling not reported");
  } else if (name == "baseline_vel200_startup") {
    handleCommand("VEL,200,0");
    mockMillis = 20;
    updateVelocityControl();
    outputs(0, 92, 0, 0);
    logical(92, 0);
    require(leftIntegralPwm > 0.79f && leftIntegralPwm < 0.81f,
            "Baseline startup integral changed");
  } else if (name == "baseline_vel200_no_motion_ramp") {
    handleCommand("VEL,200,0");
    for (mockMillis = 20; mockMillis <= 200; mockMillis += 20) {
      updateVelocityControl();
      require(currentLeftPwm < 100, "Baseline reached ceiling before expected");
    }
    mockMillis = 220;
    updateVelocityControl();
    logical(100, 0);
    for (mockMillis = 240; mockMillis <= 1500; mockMillis += 20) {
      updateVelocityControl();
      require(currentLeftPwm <= MAX_SAFE_PWM, "VEL exceeded new ceiling");
    }
    logical(136, 0); // Existing FF75 + P16 + integral bound45, not forced153.
  } else if (name == "candidate_ff85_configuration") {
    handleCommand("FF,85,60,75,60");
    outputs(0, 0, 0, 0);
    require(controlMode == CONTROL_STOPPED && !motorsActive,
            "FF configuration energized a motor");
    require(leftForwardStaticPwm == 85 && leftReverseStaticPwm == 60 &&
            rightForwardStaticPwm == 75 && rightReverseStaticPwm == 60,
            "Candidate changed an unrelated feed-forward value");
    require(velocityKp == 0.080f && velocityKi == 0.200f && velocityKv == 0.000f,
            "Candidate changed PI gains");
    require(Serial.output.str() == "OK,FF,85,60,75,60\n", "Missing FF ack");
  } else if (name == "candidate_ff85_vel200_startup") {
    handleCommand("FF,85,60,75,60");
    handleCommand("VEL,200,0");
    mockMillis = 20;
    updateVelocityControl();
    outputs(0, 102, 0, 0);
    logical(102, 0);
    require(targetLeftTicksPerSecond == 200 && targetRightTicksPerSecond == 0,
            "Candidate changed velocity targets");
  } else if (name == "candidate_ff85_feedback_adjusts_output") {
    handleCommand("FF,85,60,75,60");
    handleCommand("VEL,200,0");
    mockMillis = 20;
    updateVelocityControl();
    logical(102, 0);
    // Synthetic 10 ticks in the next 20 ms: filtered speed becomes 175.
    encoderA_raw = 10;
    mockMillis = 40;
    updateVelocityControl();
    logical(88, 0);
    require(measuredLeftTicksPerSecond == 175.0f,
            "Synthetic feedback calculation changed");
  } else if (name == "candidate_ff85_refresh_preserves_integral") {
    handleCommand("FF,85,60,75,60");
    handleCommand("VEL,200,0");
    mockMillis = 20;
    updateVelocityControl();
    handleCommand("VEL,200,0");
    require(leftIntegralPwm > 0.79f && leftIntegralPwm < 0.81f,
            "Refresh reset integral");
    mockMillis = 40;
    updateVelocityControl();
    require(leftIntegralPwm > 1.59f && leftIntegralPwm < 1.61f,
            "Integral did not accumulate across refresh");
  } else if (name == "candidate_ff85_other_directions_unchanged") {
    handleCommand("FF,85,60,75,60");
    handleCommand("VEL,0,200");
    mockMillis = 20;
    updateVelocityControl();
    outputs(0, 0, 92, 0);
    logical(0, 92);
    handleCommand("STOP");
    handleCommand("VEL,-200,0");
    mockMillis = 40;
    updateVelocityControl();
    outputs(77, 0, 0, 0);
    logical(-77, 0);
    handleCommand("STOP");
    handleCommand("VEL,0,-200");
    mockMillis = 60;
    updateVelocityControl();
    outputs(0, 0, 0, 77);
    logical(0, -77);
  } else if (name == "candidate_ff85_active_change_rejected") {
    handleCommand("FF,85,60,75,60");
    handleCommand("VEL,200,0");
    mockMillis = 20;
    updateVelocityControl();
    Serial.clearOutput();
    handleCommand("FF,75,60,75,60");
    require(Serial.output.str() == "ERR,MOTORS_ACTIVE\n",
            "Active FF reconfiguration was allowed");
    require(leftForwardStaticPwm == 85, "Rejected FF command changed profile");
    handleCommand("STOP");
    outputs(0, 0, 0, 0);
  } else if (name == "candidate_ff85_stop_and_restore") {
    handleCommand("FF,85,60,75,60");
    handleCommand("VEL,200,0");
    mockMillis = 20;
    updateVelocityControl();
    handleCommand("STOP");
    outputs(0, 0, 0, 0);
    require(targetLeftTicksPerSecond == 0 && leftIntegralPwm == 0,
            "Candidate STOP did not clear control state");
    // Restoration is an explicit stopped-only command, not an automatic
    // consequence of STOP. A future hardware helper must confirm its ack.
    handleCommand("FF,75,60,75,60");
    require(leftForwardStaticPwm == 75, "Baseline FF not restored");
    handleCommand("VEL,200,0");
    mockMillis = 40;
    updateVelocityControl();
    logical(92, 0);
  } else if (name == "candidate_ff85_watchdog_and_limits") {
    handleCommand("FF,85,60,75,60");
    handleCommand("VEL,200,0");
    mockMillis = 20;
    updateVelocityControl();
    require(MAX_SAFE_PWM == 153 && MOTOR_WATCHDOG_MS == 350,
            "Candidate changed limits");
    mockMillis = 351;
    loop();
    outputs(0, 0, 0, 0);
    require(controlMode == CONTROL_STOPPED && leftIntegralPwm == 0,
            "Candidate watchdog did not stop");
    Serial.clearOutput();
    handleCommand("INFO");
    require(Serial.output.str() == "INFO,2,748,153,2000\n", "INFO changed");
  } else if (name == "candidate_ff85_range_rejection") {
    handleCommand("FF,85,60,75,60");
    handleCommand("FF,154,60,75,60");
    require(leftForwardStaticPwm == 85, "Out-of-range FF was accepted");
    outputs(0, 0, 0, 0);
    require(Serial.output.str().find("ERR,RANGE") != std::string::npos,
            "FF range rejection disappeared");
  } else if (name == "kp160_pid_resets_ff_then_full_ff_applied") {
    handleCommand("FF,85,60,75,60");
    Serial.clearOutput();
    handleCommand("PID,160,200,0,75");
    outputs(0, 0, 0, 0);
    require(velocityKp == 0.160f && velocityKi == 0.200f && velocityKv == 0.000f,
            "Candidate PID mismatch");
    require(leftForwardStaticPwm == 75 && leftReverseStaticPwm == 75 &&
            rightForwardStaticPwm == 75 && rightReverseStaticPwm == 75,
            "PID feed-forward reset side effect changed");
    require(Serial.output.str() == "OK,PID,160,200,0,75\n", "Missing PID ack");
    handleCommand("FF,85,60,75,60");
    require(leftForwardStaticPwm == 85 && leftReverseStaticPwm == 60 &&
            rightForwardStaticPwm == 75 && rightReverseStaticPwm == 60,
            "Full candidate FF values not reapplied after PID");
    require(!motorsActive && controlMode == CONTROL_STOPPED,
            "PID/FF configuration powered motors");
  } else if (name == "kp160_startup_same_ceiling_and_unpowered_other_wheel") {
    handleCommand("PID,160,200,0,75");
    handleCommand("FF,85,60,75,60");
    handleCommand("VEL,200,0");
    mockMillis = 20;
    updateVelocityControl();
    logical(118, 0);
    outputs(0, 118, 0, 0);
    require(leftIntegralPwm > 0.79f && leftIntegralPwm < 0.81f,
            "Candidate changed integral gain");
  } else if (name == "ff85_reference_overspeed_output") {
    handleCommand("FF,85,60,75,60");
    handleCommand("VEL,200,0");
    mockMillis = 20;
    updateVelocityControl();
    encoderA_raw = 20;  // Synthetic raw1000, filtered350 ticks/s.
    mockMillis = 40;
    updateVelocityControl();
    require(measuredLeftTicksPerSecond == 350.0f, "Synthetic filtered speed changed");
    logical(73, 0);  // 85 - .080*150 + (.8 - .6).
  } else if (name == "kp160_overspeed_reduces_drive_more_than_reference") {
    handleCommand("PID,160,200,0,75");
    handleCommand("FF,85,60,75,60");
    handleCommand("VEL,200,0");
    mockMillis = 20;
    updateVelocityControl();
    encoderA_raw = 20;
    mockMillis = 40;
    updateVelocityControl();
    require(measuredLeftTicksPerSecond == 350.0f, "Synthetic filtered speed changed");
    logical(61, 0);  // 85 - .160*150 + (.8 - .6).
    outputs(0, 61, 0, 0);
  } else if (name == "kp160_active_pid_change_rejected") {
    handleCommand("PID,160,200,0,75");
    handleCommand("FF,85,60,75,60");
    handleCommand("VEL,200,0");
    mockMillis = 20;
    updateVelocityControl();
    Serial.clearOutput();
    handleCommand("PID,80,200,0,75");
    require(Serial.output.str() == "ERR,MOTORS_ACTIVE\n", "Active PID change allowed");
    require(velocityKp == 0.160f && leftForwardStaticPwm == 85,
            "Rejected PID changed candidate profile");
    handleCommand("STOP");
    outputs(0, 0, 0, 0);
  } else if (name == "kp160_stop_and_full_baseline_restore") {
    handleCommand("PID,160,200,0,75");
    handleCommand("FF,85,60,75,60");
    handleCommand("VEL,200,0");
    mockMillis = 20;
    updateVelocityControl();
    handleCommand("STOP");
    handleCommand("PID,80,200,0,75");
    handleCommand("FF,75,60,75,60");
    outputs(0, 0, 0, 0);
    require(velocityKp == 0.080f && velocityKi == 0.200f && velocityKv == 0.000f,
            "Baseline PID not restored");
    require(leftForwardStaticPwm == 75 && leftReverseStaticPwm == 60 &&
            rightForwardStaticPwm == 75 && rightReverseStaticPwm == 60,
            "Baseline directional FF not restored");
    handleCommand("VEL,200,0");
    mockMillis = 40;
    updateVelocityControl();
    logical(92, 0);
  } else if (name == "kp160_watchdog_and_limits_unchanged") {
    handleCommand("PID,160,200,0,75");
    handleCommand("FF,85,60,75,60");
    handleCommand("VEL,200,0");
    mockMillis = 20;
    updateVelocityControl();
    require(MAX_SAFE_PWM == 153 && MOTOR_WATCHDOG_MS == 350 &&
            MAX_TARGET_TICKS_PER_SECOND == 2000,
            "Candidate changed safety limits");
    mockMillis = 351;
    loop();
    outputs(0, 0, 0, 0);
    require(controlMode == CONTROL_STOPPED && leftIntegralPwm == 0,
            "Candidate watchdog did not reset output/integral");
  } else if (name == "pwm153_boundaries_and_polarity") {
    handleCommand("PWM,153,0");
    outputs(0, 153, 0, 0);
    logical(153, 0);
    handleCommand("STOP");
    handleCommand("PWM,-153,0");
    outputs(153, 0, 0, 0);
    logical(-153, 0);
    handleCommand("STOP");
    handleCommand("PWM,0,153");
    outputs(0, 0, 153, 0);
    logical(0, 153);
    handleCommand("STOP");
    handleCommand("PWM,0,-153");
    outputs(0, 0, 0, 153);
    logical(0, -153);
  } else if (name == "pwm153_all_out_of_range_stop") {
    for (const char *command : {"PWM,154,0", "PWM,-154,0",
                               "PWM,0,154", "PWM,0,-154",
                               "PWM,204,0", "PWM,255,0"}) {
      handleCommand("PWM,153,0");
      Serial.clearOutput();
      handleCommand(command);
      outputs(0, 0, 0, 0);
      require(controlMode == CONTROL_STOPPED && !motorsActive,
              "Range error did not stop");
      require(Serial.output.str() == "ERR,RANGE\n", "Missing range error");
    }
  } else if (name == "pwm153_stop_watchdog_and_defaults") {
    require(velocityKp == .080f && velocityKi == .200f && velocityKv == 0.f,
            "Flash changed default gains");
    require(leftForwardStaticPwm == 75 && leftReverseStaticPwm == 60 &&
            rightForwardStaticPwm == 75 && rightReverseStaticPwm == 60,
            "Flash changed default FF");
    handleCommand("PWM,153,0");
    mockMillis = 351;
    loop();
    outputs(0, 0, 0, 0);
    require(controlMode == CONTROL_STOPPED && !motorsActive,
            "PWM153 watchdog failed");
    handleCommand("PWM,153,0");
    handleCommand("STOP");
    outputs(0, 0, 0, 0);
  } else if (name == "pwm153_vel_clamp_and_zero_channel") {
    handleCommand("VEL,2000,0");
    mockMillis = 20;
    updateVelocityControl();
    outputs(0, 153, 0, 0);
    logical(153, 0);
    handleCommand("STOP");
    handleCommand("VEL,0,-2000");
    mockMillis = 40;
    updateVelocityControl();
    outputs(0, 0, 0, 153);
    logical(0, -153);
  } else {
    throw std::runtime_error("Unknown test");
  }
}

int main(int argc, char **argv) {
  try {
    if (argc != 2) throw std::runtime_error("Expected one test name");
    test(argv[1]);
    std::cout << "PASS " << argv[1] << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
