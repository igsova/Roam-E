"""Promoted production sketch (14 Oct): V3/P230B3 cases + production defaults; no hardware."""
from pathlib import Path
import subprocess
import unittest
from test_bounded_startup_firmware import compile_case
from test_response_id_firmware import V3_CASES
ROOT = Path(__file__).resolve().parents[1]


class ProductionFirmwareTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compile_case(cls, ROOT / 'firmware/tests/test_production_controller.cpp')

    def test_production(self):
        for case in (*V3_CASES, 'p230_ceiling', 'production_defaults',
                     'production_normal_pivot', 'production_direct_pwm_range',
                     'bk_pivot_stall_ramp_cap_timeout', 'bk_release_on_motion',
                     'bk_wrong_direction_not_progress', 'bk_moving_wheel_no_boost',
                     'bk_slow_target_window', 'bk_stop_and_watchdog_clear',
                     'bk_opposite_target_resets'):
            with self.subTest(case=case):
                r = subprocess.run([self.binary, case], capture_output=True, text=True, timeout=2)
                self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
                self.assertEqual(r.stdout.strip(), 'PASS ' + case)


if __name__ == '__main__':
    unittest.main()
