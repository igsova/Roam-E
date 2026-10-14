#include "Arduino.h"
#include <iostream>
#include <stdexcept>
#include <string>
#include "../roam_e_motor_controller/roam_e_motor_controller.ino"
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
    require(Serial.output.str()=="CAPS,BOUNDED_DUAL_V3_P230B3,128,153,230,300,250,1000,6000,1400,160,3\n","Wrong P230 caps");
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
  } else if (name=="handoff_drops_boost") {
    for (int sign : {-1,1}) {
      for (const char *pid : {"PID,240,600,0,85","PID,320,400,0,85","PID,200,800,0,85"}) {
        mockMillis=0; encoderA_raw=encoderB_raw=0;
        armed(); handleCommand(pid);
        handleCommand(sign>0 ? "VEL,200,200" : "VEL,-200,-200");
        tick(20,0,0); tick(40,sign*4,sign*4); tick(60,sign*8,sign*8);
        require(boundedLeft.phase==BOUNDED_STARTING && boundedRight.phase==BOUNDED_STARTING,
                "RUN without two-frame proof");
        tick(80,sign*12,sign*12);
        require(boundedLeft.phase==BOUNDED_RUNNING && boundedRight.phase==BOUNDED_RUNNING,
                "No RUN at independent proof");
        require(leftIntegralPwm==0 && rightIntegralPwm==0,"Breakaway boost retained in integral");
        const float expected=85+velocityKp*(200-sign*measuredLeftTicksPerSecond);
        require(sign*currentLeftPwm==(int)(expected+.5f) &&
                sign*currentRightPwm==(int)(expected+.5f),
                "Handoff not feed-forward plus proportional feedback");
        require(sign*currentLeftPwm<128 && sign*currentRightPwm<128,"Boost not released");
        tick(100,sign*24,sign*24);
        require(sign*currentLeftPwm>=0 && sign*currentRightPwm>=0,"Handoff reversed drive");
        handleCommand("STOP"); stopped();
      }
    }
  } else if (name=="handoff_preserves_startup_ramp") {
    armed(); handleCommand("VEL,200,200"); tick(20,0,0);
    require(currentLeftPwm==128 && currentRightPwm==128,"Initial boost changed");
    tick(40,0,0); require(currentLeftPwm==133 && currentRightPwm==133,"Ramp changed");
    tick(60,0,0); require(currentLeftPwm==138 && currentRightPwm==138,"Ramp changed");
    require(leftIntegralPwm==0 && rightIntegralPwm==0,"Startup integral changed");
  } else if (name=="handoff_restart_clean") {
    running(); leftIntegralPwm=rightIntegralPwm=40;
    handleCommand("STOP"); stopped(); handleCommand("START,0"); handleCommand("START,1");
    mockMillis=0; encoderA_raw=encoderB_raw=0;
    handleCommand("VEL,200,200"); tick(20,0,0); tick(40,4,4); tick(60,8,8); tick(80,12,12);
    require(leftIntegralPwm==0 && rightIntegralPwm==0,"Restart inherited boost/integral");
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
    tick(20,0,0); tick(40,name=="wrong_direction_left" ? -2 : 4,name=="wrong_direction_right" ? -2 : 4);
    require(!boundedFaultLatched && motorsActive,"Two-tick wobble faulted");
    tick(60,name=="wrong_direction_left" ? -3 : 8,name=="wrong_direction_right" ? -3 : 8);
    stopped(); require(boundedFaultLatched && boundedFaultCode==4,"Three backward ticks not latched");
    handleCommand("STOP"); mockMillis=0; encoderA_raw=encoderB_raw=0;
    handleCommand("VEL,200,200"); tick(20,0,0); tick(40,4,4); tick(60,8,8); tick(80,12,12);
    tick(100,name=="wrong_direction_left" ? 9 : 16,name=="wrong_direction_right" ? 9 : 16);
    stopped(); require(boundedFaultLatched && boundedFaultCode==4,"Three-tick backward step while running not latched");
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
    require(leftIntegralPwm==10 && rightIntegralPwm==10 && currentLeftPwm<=230 && currentRightPwm<=230,
            "Cap antiwindup failed");
    leftIntegralPwm=rightIntegralPwm=-45;
    measuredLeftTicksPerSecond=measuredRightTicksPerSecond=900; tick(120,14,14);
    require(currentLeftPwm==0 && currentRightPwm==0 && leftIntegralPwm==-45 && rightIntegralPwm==-45,
            "Zero-bound antiwindup/reversal failed");
  } else if (name=="wrap_clock_counts") {
    armed(); mockMillis=0xfffffff0UL; encoderA_raw=2147483646L; encoderB_raw=-2147483646L;
    handleCommand("VEL,200,200"); tick(0x100000004UL,-2147483646L,-2147483646L);
    require(!boundedFaultLatched && currentLeftPwm==128 && currentRightPwm==128,"Rollover pair startup");
  } else if (name=="p230_ceiling") {
    setup(); Serial.clearOutput(); handleCommand("INFO");
    require(Serial.output.str()=="INFO,2,748,230,2000\n","INFO ceiling");
    armed(); handleCommand("VEL,200,200");
    for (unsigned long t=20;t<=280;t+=20) tick(t,0,0);
    require(currentLeftPwm==153 && currentRightPwm==153,"Startup ramp exceeded 153");
    handleCommand("STOP"); mockMillis=0; encoderA_raw=encoderB_raw=0;
    armed(); handleCommand("VEL,200,200"); tick(20,0,0); tick(40,4,4); tick(60,8,8); tick(80,12,12);
    velocityKp=1; leftIntegralPwm=rightIntegralPwm=45; measuredLeftTicksPerSecond=measuredRightTicksPerSecond=0;
    tick(100,13,13);
    require(currentLeftPwm>153 && currentLeftPwm<=230 && currentRightPwm<=230,"Running output not allowed above 153 up to 230");
    handleCommand("STOP"); setup(); handleCommand("PWM,231,0"); require(!motorsActive,"231 accepted");
  } else if (name=="production_defaults") {
    setup(); stopped(); require(!boundedStartupEnabled,"Opt-in mode not default off");
    require(velocityKp>.199f && velocityKp<.201f && velocityKi>.799f && velocityKi<.801f &&
            velocityKv>.089f && velocityKv<.091f,"Default PI/Kv");
    require(leftForwardStaticPwm==70 && leftReverseStaticPwm==55 &&
            rightForwardStaticPwm==42 && rightReverseStaticPwm==63,"Default static PWM");
    require(MAX_SAFE_PWM==230 && MOTOR_WATCHDOG_MS==350 && MAX_TARGET_TICKS_PER_SECOND==2000,"Limits");
  } else if (name=="production_normal_pivot") {
    // ROS uses START,0 normal PI: opposite-sign targets and per-direction FF + Kv.
    setup(); handleCommand("VEL,-300,300"); tick(20,0,0);
    const int left=(int)(-55-.09f*300-.2f*300-.8f*300*.02f-.5f);
    require(currentLeftPwm<0 && currentRightPwm>0,"Pivot signs");
    require(currentLeftPwm>=left-1 && currentLeftPwm<=left+1,"Left reverse FF+Kv+PI");
    mockMillis=371; loop(); stopped();
    require(Serial.output.str().find("EVENT,WATCHDOG_STOP")!=std::string::npos,"Watchdog");
  } else if (name=="production_direct_pwm_range") {
    setup(); handleCommand("PWM,230,-230"); require(currentLeftPwm==230 && currentRightPwm==-230,"230 refused");
    require(mockOutputs[LEFT_IN2]==230 && mockOutputs[RIGHT_IN2]==230,"R19 polarity at 230");
    handleCommand("PWM,231,0"); stopped(); handleCommand("STOP");
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
