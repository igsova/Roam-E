"""Actual ID_V1 candidate logic tests; no USB, flash or physical/AVR timing claims."""
from pathlib import Path
import subprocess
import unittest
import test_motor_firmware
from test_bounded_startup_firmware import compile_case

ROOT = Path(__file__).resolve().parents[1]
SKETCH = ROOT / 'firmware/candidates/r19_response_id_20261013/roam_e_motor_controller.ino'
V3_CASES = ('default_disabled_caps', 'paired_forward', 'paired_reverse', 'paired_handoff',
            'target_shape', 'no_response_both', 'no_response_left', 'no_response_right',
            'stall_left', 'stall_right', 'wrong_direction_left', 'wrong_direction_right',
            'overspeed_left', 'overspeed_right', 'pair_separation', 'travel_budget_left',
            'travel_budget_right', 'six_second_deadline', 'refresh_preserves_start',
            'active_reversal_stop', 'direct_pwm_refused', 'active_config_refused',
            'stop_disable_restore', 'watchdog', 'timing_gap', 'antiwindup_both',
            'wrap_clock_counts', 'phase_diagnostic', 'damped_pair_handoff',
            'balanced_pair_handoff', 'handoff_drops_boost',
            'handoff_preserves_startup_ramp', 'handoff_restart_clean')
ID_CASES = ('id_duty_forward', 'id_duty_reverse', 'id_duty_step', 'id_duty_shape',
            'id_requires_start', 'id_reversal_and_mixing', 'id_duty_guards',
            'id_watchdog_and_deadline', 'id_timer1_ab', 'id_step_line', 'id_edge_isr')


class CandidateRegressionTest(test_motor_firmware.MotorFirmwareTest):
    @classmethod
    def setUpClass(cls):
        compile_case(cls, ROOT / 'firmware/tests/test_motor_controller.cpp',
                     ('-DROAM_MOTOR_SKETCH="' + str(SKETCH) + '"',))


class CandidateResponseIdTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compile_case(cls, ROOT / 'firmware/tests/test_response_id.cpp')

    def run_cases(self, cases):
        for case in cases:
            with self.subTest(case=case):
                r = subprocess.run([self.binary, case], capture_output=True, text=True, timeout=2)
                self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
                self.assertEqual(r.stdout.strip(), 'PASS ' + case)

    def test_v3_guards_retained(self):
        self.run_cases(V3_CASES)

    def test_identification_mode(self):
        self.run_cases(ID_CASES)


if __name__ == '__main__':
    unittest.main()
