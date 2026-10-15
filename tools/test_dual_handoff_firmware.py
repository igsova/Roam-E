"""Actual dual candidate logic tests; no USB or physical/AVR timing claims."""
from pathlib import Path
import unittest
import subprocess
import test_motor_firmware
from test_bounded_startup_firmware import compile_case
ROOT=Path(__file__).resolve().parents[1]
SKETCH=ROOT/'firmware/candidates/r19_dual_handoff_20261013/roam_e_motor_controller.ino'
class CandidateRegressionTest(test_motor_firmware.MotorFirmwareTest):
    @classmethod
    def setUpClass(cls):
        compile_case(cls,ROOT/'firmware/tests/test_motor_controller.cpp',
                     ('-DROAM_MOTOR_SKETCH="'+str(SKETCH)+'"',))
class CandidateDualHoldTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compile_case(cls,ROOT/'firmware/tests/test_dual_handoff.cpp')
    def test_dual_guards_and_hold(self):
        cases=('default_disabled_caps','paired_forward','paired_reverse','paired_handoff',
               'target_shape','no_response_both','no_response_left','no_response_right',
               'stall_left','stall_right','wrong_direction_left','wrong_direction_right',
               'overspeed_left','overspeed_right','pair_separation','travel_budget_left',
               'travel_budget_right','six_second_deadline','refresh_preserves_start',
               'active_reversal_stop','direct_pwm_refused','active_config_refused',
               'stop_disable_restore','watchdog','timing_gap','antiwindup_both',
               'wrap_clock_counts','phase_diagnostic','damped_pair_handoff',
               'balanced_pair_handoff','handoff_drops_boost',
               'handoff_preserves_startup_ramp','handoff_restart_clean')
        for case in cases:
            with self.subTest(case=case):
                r=subprocess.run([self.binary,case],capture_output=True,text=True,timeout=2)
                self.assertEqual(r.returncode,0,r.stdout+r.stderr)
                self.assertEqual(r.stdout.strip(),'PASS '+case)
if __name__=='__main__': unittest.main()
