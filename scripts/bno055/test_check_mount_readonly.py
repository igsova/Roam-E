import importlib.util
import math
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'mount_check', Path(__file__).with_name('check_mount_readonly.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class MountCheckTests(unittest.TestCase):
    def test_signed_decode(self):
        self.assertEqual(module.signed16(255, 255), -1)
        self.assertEqual(module.signed16(0, 128), -32768)
        self.assertEqual(module.signed16(255, 127), 32767)

    def test_vector_scaling(self):
        self.assertEqual(module.vector([100, 0, 156, 255, 0, 0], 0, .01),
                         [1, -1, 0])

    def test_summary(self):
        s = module.summarize([[1, 2, 3], [3, 4, 5]])
        self.assertEqual(s['mean'], [2, 3, 4])
        self.assertAlmostEqual(s['stddev'][0], math.sqrt(2))

    def test_reject_changed_mapping(self):
        registers = {'0x00': '0xa0', '0x3b': '0x00', '0x3d': '0x08',
                     '0x41': '0x24', '0x42': '0x00'}
        module.check_registers(registers)
        registers['0x42'] = '0x01'
        with self.assertRaises(RuntimeError):
            module.check_registers(registers)

    def test_quaternion_summary(self):
        self.assertEqual(module.summarize([[1, 0, 0, 0], [1, 0, 0, 0]])['mean'],
                         [1, 0, 0, 0])


if __name__ == '__main__':
    unittest.main()
