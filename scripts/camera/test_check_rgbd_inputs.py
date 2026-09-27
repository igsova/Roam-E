import unittest

from check_rgbd_inputs import distribution, nearest_deltas_ms


class TimingHelpersTest(unittest.TestCase):
    def test_empty_inputs(self):
        self.assertIsNone(distribution([]))
        self.assertEqual(nearest_deltas_ms([], [1]), [])
        self.assertEqual(nearest_deltas_ms([1], []), [])

    def test_distribution_sorts_without_changing_input(self):
        values = [3, 1, 2]
        self.assertEqual(distribution(values), {'min': 1, 'median': 2, 'p95': 3, 'max': 3})
        self.assertEqual(values, [3, 1, 2])

    def test_nearest_pair_sign_and_overlap_edges(self):
        values = nearest_deltas_ms([-1, 0, 0.49, 0.51, 1, 2], [1, 0])
        self.assertEqual(len(values), 4)
        for actual, expected in zip(values, [0, -490, 490, 0]):
            self.assertAlmostEqual(actual, expected)

    def test_single_timestamp(self):
        self.assertEqual(nearest_deltas_ms([0, 1, 2], [1]), [0])


if __name__ == '__main__':
    unittest.main()
