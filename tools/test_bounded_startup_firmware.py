"""Offline actual-sketch tests only; no hardware access/flash/physics claims."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import test_motor_firmware

ROOT = Path(__file__).resolve().parents[1]
SKETCH = ROOT / 'firmware/candidates/r19_bounded_start_20261013/roam_e_motor_controller.ino'

def compile_case(cls, source, extra=()):
    compiler = shutil.which('clang++') or shutil.which('g++')
    if not compiler:
        raise unittest.SkipTest('Host C++ compiler required')
    temp = tempfile.TemporaryDirectory(prefix='roam-bounded-start-test-')
    cls.addClassCleanup(temp.cleanup)
    cls.binary = str(Path(temp.name) / 'test')
    subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-I', str(ROOT / 'firmware/tests'), *extra, str(source),
                    '-o', cls.binary], check=True, capture_output=True, text=True, timeout=30)

class CandidateRegressionTest(test_motor_firmware.MotorFirmwareTest):
    @classmethod
    def setUpClass(cls):
        compile_case(cls, ROOT / 'firmware/tests/test_motor_controller.cpp',
                     ('-DROAM_MOTOR_SKETCH="'+str(SKETCH)+'"',))

class CandidateStartupTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compile_case(cls, ROOT / 'firmware/tests/test_bounded_startup.cpp')

    def test_opt_in_startup_guards_and_handoff(self):
        cases = ('disabled_and_caps', 'four_directions', 'ramp_cap_no_response_latch',
                 'handoff_and_speed_feedback', 'progress_loss_latch', 'wrong_direction',
                 'overspeed', 'timing_gap', 'refresh_does_not_restart',
                 'active_config_refused', 'stop_disable_restore', 'invalid_targets_stop',
                 'reversal_requires_stop', 'direct_pwm_refused', 'watchdog',
                 'antiwindup_direction_and_cap', 'episode_deadline',
                 'millis_and_counter_wrap', 'diagnostic_phase',
                 'responsive_four_direction_handoff')
        for case in cases:
            with self.subTest(case=case):
                result = subprocess.run([self.binary, case], capture_output=True,
                                        text=True, timeout=2)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                self.assertEqual(result.stdout.strip(),'PASS '+case)

if __name__ == '__main__':
    unittest.main()
