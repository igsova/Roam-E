"""Host-compile the PWM153-era Nano sketch (archived 14 Oct); never opens a device.

The promoted production sketch is tested by tools/test_production_firmware.py.

This tests pin-output logic, not AVR timing/widths, bootloader behavior, physical
direction, electrical safety or a firmware image currently flashed on a board.
Requires a C++17 host compiler (clang++ or g++). A real AVR build is separate.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class MotorFirmwareTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('clang++') or shutil.which('g++')
        if compiler is None:
            raise unittest.SkipTest('A host C++ compiler is required')
        root = Path(__file__).resolve().parents[1]
        temporary = tempfile.TemporaryDirectory(prefix='roam-motor-firmware-test-')
        cls.addClassCleanup(temporary.cleanup)
        cls.binary = str(Path(temporary.name) / 'test_motor_controller')
        subprocess.run([
            compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
            '-I', str(root / 'firmware/tests'),
            str(root / 'firmware/tests/test_motor_controller.cpp'),
            '-o', cls.binary,
        ], check=True, capture_output=True, text=True, timeout=30)

    def test_firmware_output_and_safety_logic(self):
        cases = (
            'pins_and_limits', 'startup_stopped', 'left_positive', 'right_positive',
            'left_negative', 'right_negative', 'clamp_before_polarity',
            'zero_channel', 'stop', 'zero_command', 'reversal_dead_time',
            'watchdog', 'watchdog_boundary', 'encoder_normalization',
            'logical_telemetry', 'velocity_forward', 'velocity_reverse',
            'zero_velocity', 'invalid_command', 'range_rejection', 'info_compatible',
            'baseline_vel200_startup', 'baseline_vel200_no_motion_ramp',
            'candidate_ff85_configuration', 'candidate_ff85_vel200_startup',
            'candidate_ff85_feedback_adjusts_output',
            'candidate_ff85_refresh_preserves_integral',
            'candidate_ff85_other_directions_unchanged',
            'candidate_ff85_active_change_rejected', 'candidate_ff85_stop_and_restore',
            'candidate_ff85_watchdog_and_limits', 'candidate_ff85_range_rejection',
            'kp160_pid_resets_ff_then_full_ff_applied',
            'kp160_startup_same_ceiling_and_unpowered_other_wheel',
            'ff85_reference_overspeed_output',
            'kp160_overspeed_reduces_drive_more_than_reference',
            'kp160_active_pid_change_rejected',
            'kp160_stop_and_full_baseline_restore',
            'kp160_watchdog_and_limits_unchanged',
            'pwm153_boundaries_and_polarity', 'pwm153_all_out_of_range_stop',
            'pwm153_stop_watchdog_and_defaults', 'pwm153_vel_clamp_and_zero_channel',
        )
        for case in cases:
            with self.subTest(case=case):
                result = subprocess.run(
                    [self.binary, case], capture_output=True, text=True, timeout=2,
                )
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertEqual(result.stdout.strip(), 'PASS ' + case)


if __name__ == '__main__':
    unittest.main()
