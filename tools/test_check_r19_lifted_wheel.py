import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('lifted', Path(__file__).with_name('check_r19_lifted_wheel.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class LiftedWheelTest(unittest.TestCase):
    def test_one_channel_only(self):
        self.assertEqual(module.pulse_command('left', True), b'PWM,40,0\n')
        self.assertEqual(module.pulse_command('right', True), b'PWM,0,40\n')

    def test_unarmed_refused(self):
        for side in ('left', 'right', 'both', None):
            with self.assertRaises(ValueError):
                module.pulse_command(side, False)

    def test_invalid_channel_refused(self):
        with self.assertRaises(ValueError):
            module.pulse_command('both', True)

    def test_telemetry(self):
        self.assertEqual(module.parse_telemetry('T,1000,4,-3,0,0,0,0,0,0'), [1000,4,-3,0,0,0,0,0,0])
        self.assertIsNone(module.parse_telemetry('OK,STOP'))

    def test_bad_telemetry_refused(self):
        for line in ('T,1,2,3,0,0', 'T,x,0,0,0,0,0,0,0,0',
                     'T,-1,0,0,0,0,0,0,0,0', 'T,2,0,0,101,0,0,0,0,0',
                     'T,2,2147483648,0,0,0,0,0,0,0'):
            with self.assertRaises(ValueError):
                module.parse_telemetry(line)

    def test_resting(self):
        rows = [[t,4,7,0,0,0,0,0,0] for t in (100,150,200,250,300)]
        self.assertTrue(module.resting(rows))
        self.assertFalse(module.resting(rows[:4]))
        rows[-1][3] = 1
        self.assertFalse(module.resting(rows))

    def test_changing_counts_not_resting(self):
        rows = [[t,t,0,0,0,0,0,0,0] for t in (100,150,200,250,300)]
        self.assertFalse(module.resting(rows))

    def test_short_rest_not_accepted(self):
        self.assertFalse(module.resting([[t,0,0,0,0,0,0,0,0] for t in range(5)]))

    def test_targets_not_zero(self):
        self.assertFalse(module.resting([[t,0,0,0,0,1,0,0,0] for t in (100,150,200,250,300)]))

    def test_wrap_delta(self):
        self.assertEqual(module.counter_delta(-2147483648, 2147483647), 1)
        self.assertEqual(module.counter_delta(2147483647, -2147483648), -1)
        self.assertEqual(module.counter_delta(13, 10), 3)


if __name__ == '__main__':
    unittest.main()
