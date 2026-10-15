#include "Arduino.h"
#include <iostream>
#include <stdexcept>
#include <string>
#include "../candidates/r19_dual_hold_20261013/roam_e_motor_controller.ino"
void require(bool condition,const char *message) { if (!condition) throw std::runtime_error(message); }
void armed() {
  setup(); handleCommand("PID,240,600,0,85"); handleCommand("FF,85,85,85,85");
  handleCommand("START,1");
  require(boundedStartupEnabled && !motorsActive,"Arming powered motors");
}
void stopped() { require(!motorsActive && currentLeftPwm==0 && currentRightPwm==0,"Both outputs not stopped"); }
void tick(unsigned long now,long left,long right) {
  mockMillis=now; encoderA_raw=left; encoderB_raw=-right; updateVelocityControl();
}
void running(int sign=1) {
  armed(); handleCommand(sign>0 ? "VEL,200,200" : "VEL,-200,-200");
  tick(20,0,0); tick(40,sign*4,sign*4); tick(60,sign*8,sign*8); tick(80,sign*12,sign*12);
  require(boundedLeft.phase==BOUNDED_RUNNING && boundedRight.phase==BOUNDED_RUNNING,"No dual RUN proof");
}
void test(const std::string &name) {
  if (name=="default_disabled_caps") {
    setup(); stopped(); require(!boundedStartupEnabled,"Not default disabled");
    Serial.clearOutput(); handleCommand("CAPS");
    require(Serial.output.str()=="CAPS,BOUNDED_DUAL_V2,128,153,300,250,1000,6000,1400,160\n","Wrong V2 caps");
  } else if (name=="paired_forward" || name=="paired_reverse") {
    const int sign=name=="paired_forward" ? 1 : -1;
    armed(); handleCommand(sign>0 ? "VEL,200,200" : "VEL,-200,-200"); tick(20,0,0);
    require(currentLeftPwm==sign*128 && currentRightPwm==sign*128,"Pair startup signs/duty");
    require(mockOutputs[LEFT_IN1]==(sign<0 ? 128 : 0) && mockOutputs[LEFT_IN2]==(sign>0 ? 128 : 0) &&
            mockOutputs[RIGHT_IN1]==(sign>0 ? 128 : 0) && mockOutputs[RIGHT_IN2]==(sign<0 ? 128 : 0),
            "Electrical R19 pair polarity changed");
    require(leftIntegralPwm==0 && rightIntegralPwm==0,"Startup windup");
  } else if (name=="paired_handoff") {
    running(); const int beforeLeft=currentLeftPwm,beforeRight=currentRightPwm;
    tick(100,24,24);
    require(currentLeftPwm>=0 && currentLeftPwm<beforeLeft && currentRightPwm>=0 && currentRightPwm<beforeRight,
            "Dual overspeed not corrected");
    require(boundedAbs(leftIntegralPwm)<=45 && boundedAbs(rightIntegralPwm)<=45,"Integral ceiling changed");
  } else if (name=="damped_pair_handoff" || name=="balanced_pair_handoff") {
    for (int sign : {-1,1}) {
      mockMillis=0; encoderA_raw=encoderB_raw=0;
      armed(); handleCommand(name=="damped_pair_handoff" ? "PID,320,400,0,85" : "PID,200,800,0,85");
      handleCommand(sign>0 ? "VEL,200,200" : "VEL,-200,-200");
      tick(20,0,0); tick(40,sign*4,sign*4); tick(60,sign*8,sign*8); tick(80,sign*12,sign*12);
      require(boundedLeft.phase==BOUNDED_RUNNING && boundedRight.phase==BOUNDED_RUNNING,"Damped pair never RUN");
      const int beforeLeft=sign*currentLeftPwm,beforeRight=sign*currentRightPwm;
      tick(100,sign*24,sign*24);
      require(sign*currentLeftPwm>=0 && sign*currentLeftPwm<beforeLeft &&
              sign*currentRightPwm>=0 && sign*currentRightPwm<beforeRight,
              "Damped overspeed/sign control failed");
      require(boundedAbs(leftIntegralPwm)<=45 && boundedAbs(rightIntegralPwm)<=45,"Damped integral bounds");
      handleCommand("STOP"); handleCommand("START,0");
      handleCommand("PID,80,200,0,75"); handleCommand("FF,75,60,75,60"); stopped();
    }
  } else if (name=="target_shape") {
    armed();
    for (const char *cmd : {"VEL,200,0","VEL,0,200","VEL,200,-200","VEL,200,199","VEL,99,99","VEL,301,301"}) {
      handleCommand("STOP"); handleCommand(cmd); stopped(); require(boundedFaultLatched,"Shape/range accepted");
    }
  } else if (name=="no_response_both" || name=="no_response_left" || name=="no_response_right") {
    armed(); handleCommand("VEL,200,200");
    for (unsigned long t=20;t<=300;t+=20) {
      tick(t,name=="no_response_left" || name=="no_response_both" ? 0 : (long)(t/5),
             name=="no_response_right" || name=="no_response_both" ? 0 : (long)(t/5));
    }
    stopped(); require(boundedFaultLatched,"No-response not latched");
    handleCommand("VEL,200,200"); stopped();
  } else if (name=="stall_left" || name=="stall_right") {
    running();
    for (unsigned long t=100;t<=340;t+=20)
      tick(t,name=="stall_left" ? 12 : (long)(t/5),name=="stall_right" ? 12 : (long)(t/5));
    stopped(); require(boundedFaultLatched,"Single stall failed to stop pair");
  } else if (name=="wrong_direction_left" || name=="wrong_direction_right") {
    armed(); handleCommand("VEL,200,200");
    tick(20,0,0); tick(40,name=="wrong_direction_left" ? -1 : 4,name=="wrong_direction_right" ? -1 : 4);
    stopped(); require(boundedFaultLatched,"Wrong direction not latched");
  } else if (name=="overspeed_left" || name=="overspeed_right") {
    armed(); handleCommand("VEL,200,200"); tick(20,0,0);
    tick(40,name=="overspeed_left" ? 21 : 4,name=="overspeed_right" ? 21 : 4);
    stopped(); require(boundedFaultLatched,"Overspeed not latched");
  } else if (name=="pair_separation") {
    running();
    for (unsigned long t=100;t<=900 && !boundedFaultLatched;t+=20)
      tick(t,12+(long)((t-80)/5),12+(long)((t-80)*.45f));
    stopped(); require(boundedFaultLatched && boundedFaultCode==9,"Pair separation not bounded");
  } else if (name=="travel_budget_left" || name=="travel_budget_right") {
    running();
    boundedLeft.startTicks=name=="travel_budget_left" ? -1390 : -1388;
    boundedRight.startTicks=name=="travel_budget_right" ? -1390 : -1388;
    tick(100,name=="travel_budget_left" ? 16 : 12,name=="travel_budget_right" ? 16 : 12);
    stopped(); require(boundedFaultLatched && boundedFaultCode==8,"Per-wheel travel not bounded");
  } else if (name=="six_second_deadline") {
    running();
    for (unsigned long t=100;t<6000;t+=20) {
      handleCommand("VEL,200,200"); tick(t,(long)(t/5),(long)(t/5));
      require(!boundedFaultLatched,"Early deadline in bounded hold");
    }
    handleCommand("VEL,200,200"); tick(6000,1200,1200);
    stopped(); require(boundedFaultLatched,"Refresh removed six-second limit");
  } else if (name=="refresh_preserves_start") {
    armed(); handleCommand("VEL,200,200"); tick(20,0,0);
    handleCommand("VEL,200,200"); tick(40,0,0);
    require(boundedLeft.startedMs==0 && boundedRight.startedMs==0 &&
            currentLeftPwm==133 && currentRightPwm==133,"Refresh reset pair boost");
  } else if (name=="active_reversal_stop") {
    running(); handleCommand("VEL,-200,-200"); stopped(); require(boundedFaultLatched,"Pair reversal accepted");
  } else if (name=="direct_pwm_refused") {
    armed(); handleCommand("PWM,128,128"); stopped(); require(boundedFaultLatched,"Direct PWM bypass");
  } else if (name=="active_config_refused") {
    running(); handleCommand("START,0"); require(boundedStartupEnabled,"Moving disable accepted");
    handleCommand("PID,80,200,0,75"); require(velocityKp>.23f,"Moving PID accepted");
  } else if (name=="stop_disable_restore") {
    running(); handleCommand("STOP"); handleCommand("START,0");
    handleCommand("PID,80,200,0,75"); handleCommand("FF,75,60,75,60"); stopped();
    require(!boundedStartupEnabled && !boundedFaultLatched &&
            leftForwardStaticPwm==75 && rightReverseStaticPwm==60,"Baseline restore failed");
  } else if (name=="watchdog") {
    running(); mockMillis=351; loop(); stopped();
    require(Serial.output.str().find("EVENT,WATCHDOG_STOP")!=std::string::npos,"Watchdog changed");
  } else if (name=="timing_gap") {
    running(); tick(231,16,16); stopped(); require(boundedFaultLatched,"Control gap accepted");
  } else if (name=="antiwindup_both") {
    running(); velocityKp=1; velocityKi=2; leftIntegralPwm=rightIntegralPwm=10;
    measuredLeftTicksPerSecond=measuredRightTicksPerSecond=0; tick(100,13,13);
    require(leftIntegralPwm==10 && rightIntegralPwm==10 && currentLeftPwm<=153 && currentRightPwm<=153,
            "Cap antiwindup failed");
    leftIntegralPwm=rightIntegralPwm=-45;
    measuredLeftTicksPerSecond=measuredRightTicksPerSecond=900; tick(120,14,14);
    require(currentLeftPwm==0 && currentRightPwm==0 && leftIntegralPwm==-45 && rightIntegralPwm==-45,
            "Zero-bound antiwindup/reversal failed");
  } else if (name=="wrap_clock_counts") {
    armed(); mockMillis=0xfffffff0UL; encoderA_raw=2147483646L; encoderB_raw=-2147483646L;
    handleCommand("VEL,200,200"); tick(0x100000004UL,-2147483646L,-2147483646L);
    require(!boundedFaultLatched && currentLeftPwm==128 && currentRightPwm==128,"Rollover pair startup");
  } else if (name=="phase_diagnostic") {
    armed(); handleCommand("VEL,200,200"); tick(20,0,0); mockMillis=50;
    Serial.clearOutput(); publishTelemetry();
    require(Serial.output.str().find("B,50,1,1,0\n")!=std::string::npos,"Missing pair phase");
  } else throw std::runtime_error("Unknown case");
}
int main(int argc,char **argv) {
  try { if (argc!=2) throw std::runtime_error("One case required"); test(argv[1]);
        std::cout<<"PASS "<<argv[1]<<'\n'; return 0; }
  catch (const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
