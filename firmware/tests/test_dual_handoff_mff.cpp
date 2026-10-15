#include "Arduino.h"
#include <iostream>
#include <stdexcept>
#include <string>
#include "../candidates/r19_dual_handoff_20261013/roam_e_motor_controller.ino"
// Actual V3 sketch with the measured per-wheel/direction FF profile; no hardware.
void require(bool condition,const char *message) { if (!condition) throw std::runtime_error(message); }
void tick(unsigned long now,long left,long right) {
  mockMillis=now; encoderA_raw=left; encoderB_raw=-right; updateVelocityControl();
}
int main() {
  try {
    for (int sign : {1,-1}) {
      mockMillis=0; encoderA_raw=encoderB_raw=0;
      setup(); handleCommand("PID,200,800,0,85"); handleCommand("FF,88,67,57,81");
      require(leftForwardStaticPwm==88 && leftReverseStaticPwm==67 &&
              rightForwardStaticPwm==57 && rightReverseStaticPwm==81,"FF not applied after PID");
      handleCommand("START,1"); handleCommand(sign>0 ? "VEL,200,200" : "VEL,-200,-200");
      tick(20,0,0);
      require(currentLeftPwm==sign*128 && currentRightPwm==sign*128,"Startup boost changed");
      tick(40,sign*4,sign*4); tick(60,sign*8,sign*8); tick(80,sign*12,sign*12);
      require(boundedLeft.phase==BOUNDED_RUNNING && boundedRight.phase==BOUNDED_RUNNING,"No RUN");
      const float leftFf=sign>0 ? 88 : 67, rightFf=sign>0 ? 57 : 81;
      const float le=200-sign*measuredLeftTicksPerSecond, re=200-sign*measuredRightTicksPerSecond;
      const float lo=leftFf+velocityKp*le, ro=rightFf+velocityKp*re;
      require(sign*currentLeftPwm==(int)(lo+.5f) && sign*currentRightPwm==(int)(ro+.5f),
              "Handoff not per-direction FF plus P");
      require(leftIntegralPwm==0 && rightIntegralPwm==0,"Handoff integral");
      handleCommand("STOP"); handleCommand("START,0");
      handleCommand("PID,80,200,0,75"); handleCommand("FF,75,60,75,60");
      require(!motorsActive && leftForwardStaticPwm==75 && rightReverseStaticPwm==60,"Restore");
    }
    std::cout<<"PASS measured_ff\n"; return 0;
  } catch (const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
