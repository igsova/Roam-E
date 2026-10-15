#include "Arduino.h"
#include <iostream>
#include <stdexcept>
#include <string>
#include "../candidates/r19_bounded_start_20261013/roam_e_motor_controller.ino"

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
void armed() {
  setup();
  handleCommand("PID,160,200,0,85");
  handleCommand("FF,85,85,85,85");
  handleCommand("START,1");
  require(boundedStartupEnabled && !motorsActive, "Arm energized output");
}
void tick(unsigned long now, long left, long right) {
  mockMillis = now;
  encoderA_raw = left;
  encoderB_raw = -right;
  updateVelocityControl();
}
void stopped() {
  require(currentLeftPwm == 0 && currentRightPwm == 0 && !motorsActive,
          "Not de-energized");
}
void test(const std::string &name) {
  if (name == "disabled_and_caps") {
    setup();
    require(!boundedStartupEnabled, "Not default disabled");
    Serial.clearOutput(); handleCommand("CAPS");
    require(Serial.output.str() == "CAPS,BOUNDED_START_V1,128,153,300,250,1000,3000\n", "Caps mismatch");
    stopped();
  } else if (name == "four_directions") {
    armed();
    for (int channel = 0; channel < 2; ++channel) {
      for (int direction : {-1, 1}) {
        handleCommand("STOP");
        const std::string cmd = "VEL," + std::to_string(channel == 0 ? direction*200 : 0) +
          "," + std::to_string(channel == 1 ? direction*200 : 0);
        handleCommand(cmd.c_str());
        const unsigned long t = mockMillis + 20;
        tick(t, encoderA_raw, -encoderB_raw);
        require((channel == 0 ? currentLeftPwm : currentRightPwm) == direction*128,
                "Initial startup sign/duty mismatch");
        require((channel == 0 ? currentRightPwm : currentLeftPwm) == 0,
                "Unselected motor powered");
        require(mockOutputs[LEFT_IN1] == (channel == 0 && direction < 0 ? 128 : 0) &&
                mockOutputs[LEFT_IN2] == (channel == 0 && direction > 0 ? 128 : 0) &&
                mockOutputs[RIGHT_IN1] == (channel == 1 && direction > 0 ? 128 : 0) &&
                mockOutputs[RIGHT_IN2] == (channel == 1 && direction < 0 ? 128 : 0),
                "Electrical R19 polarity mismatch");
        require(leftIntegralPwm == 0 && rightIntegralPwm == 0, "Startup integral grew");
      }
    }
  } else if (name == "ramp_cap_no_response_latch") {
    armed(); handleCommand("VEL,200,0");
    for (unsigned long t=20; t<300; t+=20) {
      tick(t, 0, 0);
      require(currentLeftPwm >=128 && currentLeftPwm <=153, "Startup cap changed");
      require(leftIntegralPwm == 0, "Stalled integrator grew");
    }
    tick(300, 0, 0); stopped();
    require(boundedFaultLatched, "No response not latched");
    handleCommand("VEL,200,0"); tick(320, 0, 0); stopped();
    require(Serial.output.str().find("ERR,CONTROL_FAULT") != std::string::npos,
            "Refresh restarted after fault");
  } else if (name == "handoff_and_speed_feedback") {
    armed(); handleCommand("VEL,200,0");
    tick(20,0,0); tick(40,4,-3); tick(60,8,-6); tick(80,12,-9);
    require(boundedLeft.phase == BOUNDED_RUNNING, "No confirmed handoff");
    require(boundedRight.phase == BOUNDED_IDLE && currentRightPwm == 0,
            "Passive other roll powered");
    const int before = currentLeftPwm;
    tick(100,24,-12);
    require(currentLeftPwm < before && currentLeftPwm >=0, "Overspeed not reduced");
    require(leftIntegralPwm <=45 && leftIntegralPwm >=-45, "Integral limit changed");
  } else if (name == "responsive_four_direction_handoff") {
    for (int channel=0; channel<2; ++channel) {
      for (int direction : {-1,1}) {
        // Each direction models a fresh serial-open/reset baseline.
        mockMillis=0; encoderA_raw=0; encoderB_raw=0;
        armed(); handleCommand("PID,240,600,0,85");
        const std::string cmd="VEL,"+std::to_string(channel==0 ? direction*200 : 0)+
          ","+std::to_string(channel==1 ? direction*200 : 0);
        handleCommand(cmd.c_str());
        for (unsigned long t=20; t<=80; t+=20) {
          const long counts=direction*(long)((t-20)/5);
          tick(t,channel==0 ? counts : 0,channel==1 ? counts : 0);
        }
        require((channel==0 ? boundedLeft.phase : boundedRight.phase)==BOUNDED_RUNNING,
                "Responsive profile never handed off");
        const int before=direction*(channel==0 ? currentLeftPwm : currentRightPwm);
        tick(100,channel==0 ? direction*24 : 0,channel==1 ? direction*24 : 0);
        const int after=direction*(channel==0 ? currentLeftPwm : currentRightPwm);
        require(after>=0 && after<before && after<=153,
                "Responsive overspeed correction/sign/cap failed");
        const float integral=channel==0 ? leftIntegralPwm : rightIntegralPwm;
        require(integral>=-45 && integral<=45,"Responsive integral cap failed");
        require((channel==0 ? currentRightPwm : currentLeftPwm)==0,
                "Responsive profile powered other wheel");
        handleCommand("STOP"); handleCommand("START,0");
        handleCommand("PID,80,200,0,75"); handleCommand("FF,75,60,75,60");
        stopped(); require(!boundedStartupEnabled,"Responsive restore not disabled");
      }
    }
  } else if (name == "progress_loss_latch") {
    armed(); handleCommand("VEL,200,0");
    tick(20,0,0); tick(40,4,0); tick(60,8,0); tick(80,12,0);
    for (unsigned long t=100; t<=320; t+=20) tick(t,12,0);
    tick(340,12,0); stopped();
    require(boundedFaultLatched, "Stall not latched");
    handleCommand("VEL,200,0"); stopped();
  } else if (name == "wrong_direction") {
    armed(); handleCommand("VEL,200,0"); tick(20,0,0); tick(40,-1,0);
    stopped(); require(boundedFaultLatched, "Wrong direction not latched");
  } else if (name == "overspeed") {
    armed(); handleCommand("VEL,200,0"); tick(20,0,0); tick(40,21,0);
    stopped(); require(boundedFaultLatched, "Overspeed not latched");
  } else if (name == "timing_gap") {
    armed(); handleCommand("VEL,200,0"); tick(20,0,0); tick(171,1,0);
    stopped(); require(boundedFaultLatched, "Stale control gap not latched");
  } else if (name == "refresh_does_not_restart") {
    armed(); handleCommand("VEL,200,0"); tick(20,0,0);
    handleCommand("VEL,200,0"); tick(40,0,0);
    require(boundedLeft.startedMs ==0 && currentLeftPwm ==133,
            "Refresh reset startup ramp");
  } else if (name == "active_config_refused") {
    armed(); handleCommand("VEL,200,0"); tick(20,0,0);
    handleCommand("START,0");
    require(boundedStartupEnabled && currentLeftPwm ==128, "Active config changed");
    require(Serial.output.str().find("ERR,MOTORS_ACTIVE") != std::string::npos,
            "No active config refusal");
  } else if (name == "stop_disable_restore") {
    armed(); handleCommand("VEL,200,0"); tick(20,0,0);
    handleCommand("STOP"); handleCommand("START,0");
    handleCommand("PID,80,200,0,75"); handleCommand("FF,75,60,75,60");
    stopped(); require(!boundedStartupEnabled && !boundedFaultLatched,
                       "Stopped candidate not disabled");
    require(leftForwardStaticPwm ==75 && leftReverseStaticPwm ==60 &&
            rightForwardStaticPwm ==75 && rightReverseStaticPwm ==60,
            "Baseline FF not restored");
  } else if (name == "invalid_targets_stop") {
    armed();
    for (const char *cmd : {"VEL,99,0","VEL,301,0","VEL,200,200","VEL,0,-99"}) {
      handleCommand("STOP"); handleCommand(cmd); stopped();
      require(boundedFaultLatched, "Invalid candidate target not latched");
    }
  } else if (name == "reversal_requires_stop") {
    armed(); handleCommand("VEL,200,0"); tick(20,0,0);
    handleCommand("VEL,-200,0"); stopped();
    require(boundedFaultLatched, "Moving reversal not stopped");
  } else if (name == "direct_pwm_refused") {
    armed(); handleCommand("PWM,153,0"); stopped();
    require(boundedFaultLatched, "Direct PWM bypassed calibration guards");
  } else if (name == "watchdog") {
    armed(); handleCommand("VEL,200,0"); tick(20,0,0);
    mockMillis=351; loop(); stopped();
    require(Serial.output.str().find("EVENT,WATCHDOG_STOP") != std::string::npos,
            "Watchdog changed");
  } else if (name == "antiwindup_direction_and_cap") {
    armed(); handleCommand("VEL,200,0");
    boundedLeft.phase=BOUNDED_RUNNING; boundedLeft.responded=true;
    boundedLeft.startedMs=boundedLeft.lastProgressMs=0;
    velocityKp=1; velocityKi=2; leftIntegralPwm=10;
    tick(20,1,0);
    require(currentLeftPwm <=153 && leftIntegralPwm ==10,
            "Integrator grew into saturation");
    leftIntegralPwm=-45; measuredLeftTicksPerSecond=900;
    tick(40,2,0);
    require(currentLeftPwm ==0 && leftIntegralPwm ==-45,
            "Overspeed reversed drive or wound into zero bound");
  } else if (name == "episode_deadline") {
    armed(); handleCommand("VEL,200,0");
    for (unsigned long t=20; t<3000; t+=20) {
      handleCommand("VEL,200,0"); tick(t, (long)(t/5), 0);
    }
    handleCommand("VEL,200,0"); tick(3000,600,0); stopped();
    require(boundedFaultLatched, "3s episode deadline absent");
  } else if (name == "millis_and_counter_wrap") {
    armed(); mockMillis=0xfffffff0UL; encoderA_raw=2147483646L;
    handleCommand("VEL,200,0"); tick(0x100000004UL, -2147483646L, 0);
    require(!boundedFaultLatched && currentLeftPwm ==128,
            "Bounded timestamp/counter rollover broke startup");
  } else if (name == "diagnostic_phase") {
    armed(); handleCommand("VEL,200,0"); tick(20,0,0);
    mockMillis=50; Serial.clearOutput(); publishTelemetry();
    require(Serial.output.str().find("B,50,1,0,0\n") != std::string::npos,
            "Missing phase diagnostics");
  } else throw std::runtime_error("Unknown case");
}
int main(int argc, char **argv) {
  try {
    if (argc !=2) throw std::runtime_error("One test case required");
    test(argv[1]); std::cout << "PASS " << argv[1] << '\n'; return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
