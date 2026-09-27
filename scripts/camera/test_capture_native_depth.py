import unittest
from types import SimpleNamespace
from capture_native_depth import nearest_complete


def msg(ns):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=ns)))


class PairingTests(unittest.TestCase):
    def data(self):
        frames = {'rgb':[msg(100000000)],
                  'ir':[msg(90000000),msg(110000000)],
                  'depth':[msg(95000000),msg(115000000)]}
        return frames, {k:list(v) for k,v in frames.items()}

    def test_exact_metadata_and_nearest(self):
        frames, infos = self.data()
        picked, meta = nearest_complete(frames,infos)
        self.assertIs(picked['depth'],frames['depth'][0])
        self.assertIs(meta['rgb'],infos['rgb'][0])

    def test_missing_metadata_rejected(self):
        frames, infos = self.data()
        infos['depth'] = [msg(94000000)]
        self.assertIsNone(nearest_complete(frames,infos))

    def test_unbracketed_rejected(self):
        frames, infos = self.data()
        frames['ir'] = [msg(110000000),msg(120000000)]
        self.assertIsNone(nearest_complete(frames,infos))

    def test_large_time_gap_rejected(self):
        frames, infos = self.data()
        frames['ir'] = infos['ir'] = [msg(50000000),msg(150000000)]
        self.assertIsNone(nearest_complete(frames,infos))


if __name__ == '__main__':
    unittest.main()
