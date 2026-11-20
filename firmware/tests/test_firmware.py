"""Host-side tests for the Nano motor controller.

Compiles test_production_controller.cpp, which includes the production sketch
against a mocked Arduino API (Arduino.h), and runs every test case. Needs a
C++17 host compiler (clang++ or g++); no board is involved.

    python3 -m unittest firmware/tests/test_firmware.py
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

TESTS = Path(__file__).resolve().parent
CASES = ('default_disabled_caps', 'paired_forward', 'paired_reverse', 'paired_handoff', 'target_shape', 'no_response_both', 'no_response_left', 'no_response_right', 'stall_left', 'stall_right', 'wrong_direction_left', 'wrong_direction_right', 'overspeed_left', 'overspeed_right', 'pair_separation', 'travel_budget_left', 'travel_budget_right', 'six_second_deadline', 'refresh_preserves_start', 'active_reversal_stop', 'direct_pwm_refused', 'active_config_refused', 'stop_disable_restore', 'watchdog', 'timing_gap', 'antiwindup_both', 'wrap_clock_counts', 'phase_diagnostic', 'damped_pair_handoff', 'balanced_pair_handoff', 'handoff_drops_boost', 'handoff_preserves_startup_ramp', 'handoff_restart_clean', 'p230_ceiling', 'production_defaults', 'production_normal_pivot', 'production_direct_pwm_range', 'bk_pivot_stall_ramp_cap_timeout', 'bk_release_on_motion', 'bk_wrong_direction_not_progress', 'bk_moving_wheel_no_boost', 'bk_slow_target_window', 'bk_stop_and_watchdog_clear', 'bk_opposite_target_resets')


class MotorControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('clang++') or shutil.which('g++')
        if compiler is None:
            raise unittest.SkipTest('A host C++ compiler is required')
        temporary = tempfile.TemporaryDirectory(prefix='roam-e-firmware-test-')
        cls.addClassCleanup(temporary.cleanup)
        cls.binary = str(Path(temporary.name) / 'test_production_controller')
        subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        '-I', str(TESTS), str(TESTS / 'test_production_controller.cpp'),
                        '-o', cls.binary], check=True, capture_output=True, text=True, timeout=60)

    def test_cases(self):
        for case in CASES:
            with self.subTest(case=case):
                result = subprocess.run([self.binary, case], capture_output=True, text=True, timeout=2)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertEqual(result.stdout.strip(), 'PASS ' + case)


if __name__ == '__main__':
    unittest.main()
