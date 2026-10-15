"""Actual P230 candidate logic tests (V3 cases + 90% ceiling); no USB or physics claims."""
from pathlib import Path
import subprocess
import unittest
from test_bounded_startup_firmware import compile_case
from test_response_id_firmware import V3_CASES
ROOT = Path(__file__).resolve().parents[1]


class CandidateP230Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compile_case(cls, ROOT / 'firmware/tests/test_dual_p230.cpp')

    def test_v3_cases_and_ceiling(self):
        for case in (*V3_CASES, 'p230_ceiling'):
            with self.subTest(case=case):
                r = subprocess.run([self.binary, case], capture_output=True, text=True, timeout=2)
                self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
                self.assertEqual(r.stdout.strip(), 'PASS ' + case)


if __name__ == '__main__':
    unittest.main()
