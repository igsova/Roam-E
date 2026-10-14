"""Offline analyzer tests on saved records and a synthetic first-order plant only."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).parent))
import analyze_r19_response as a

ROOT = Path(__file__).resolve().parents[1]
SAVED = (ROOT / 'docs/measurements/R19_HANDOFF_FORWARD3_20261013.json').exists()


def synthetic(sign=1, gain=10.0, deadband=60.0, tau_ms=80.0, delay_ms=20):
    """ID_V1-shaped record from a noiseless first-order duty->speed model."""
    schedule = [[0.0, sign * 92, sign * 92], [1.5, sign * 107, sign * 107]]
    steps, ticks, speed, edge_us, history = [], [0.0, 0.0], 0.0, [0, 0], []
    for k in range(150):
        ms = 1000 + 20 * k
        duty = 92 if k < 75 else 107
        history.append(duty)
        effective = history[max(0, len(history) - 1 - delay_ms // 20)]
        target = gain * (effective - deadband)
        speed += (target - speed) * (1 - math.exp(-20 / tau_ms))
        for i in (0, 1):
            before = int(ticks[i])
            ticks[i] += speed * .02
            if int(ticks[i]) != before:
                fraction = (ticks[i] - int(ticks[i])) / max(speed * .02, 1e-9)
                edge_us[i] = int((ms - 20 * fraction) * 1000)
        steps.append({'values': [ms, sign * int(ticks[0]), sign * int(ticks[1]),
                                 edge_us[0], edge_us[1], sign * duty, sign * duty]})
    return {'targets': [sign * 200, sign * 200], 'schedule': schedule, 'steps': steps,
            'episode': 'synthetic', 'timer1_fast': False, 'reason': None}


class AnalyzerTest(unittest.TestCase):
    @unittest.skipUnless(SAVED, 'Saved Mac evidence not staged here')
    def test_saved_failed_forward3(self):
        raw = a.load(str(ROOT / 'docs/measurements/R19_HANDOFF_FORWARD3_20261013.json'))
        r = a.analyze(raw, 'forward3')
        self.assertEqual(r['telemetry_dt_ms'], (50, 51))
        right = r['wheels']['right']
        self.assertEqual(right['pin'], 'D9')
        self.assertEqual(right['pwm_min_max'][1], 112)
        self.assertLess(right['corr_avg_pwm_vs_avg_speed'], 0)

    @unittest.skipUnless(SAVED, 'Saved Mac evidence not staged here')
    def test_position_profiles_repeat(self):
        out = a.position_profiles(str(ROOT / 'docs/measurements'))
        pairs = out['pairwise_shape_correlation']
        for key in ('right:FORWARD1~FORWARD2', 'right:FORWARD2~FORWARD3',
                    'right:FORWARD3~DAMPED_FORWARD_TRACK'):
            self.assertGreater(pairs[key]['r'], .8, key)

    def test_id_step_recovers_model(self):
        for sign in (1, -1):
            with self.subTest(sign=sign):
                r = a.analyze_id(synthetic(sign))
                for side in ('left', 'right'):
                    w = r['wheels'][side]
                    self.assertAlmostEqual(w['incremental_gain_ticks_s_per_pwm'], 10, delta=.6)
                    self.assertAlmostEqual(w['speed_a'], 320, delta=8)
                    self.assertLessEqual(abs(w['step_fit']['tau_ms'] - 80), 25)
                    self.assertLessEqual(w['speed_a_edge_sd'], w['speed_a_count_sd'])

    def test_id_incomplete_record(self):
        raw = synthetic(); raw['steps'] = raw['steps'][:10]
        self.assertTrue(a.analyze_id(raw)['incomplete'])


if __name__ == '__main__':
    unittest.main()
