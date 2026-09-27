import copy
import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

import fit_rgb_ir_calibration as fit


def synthetic_data():
    obj = fit.object_grid(.035)
    kr = np.array([[580.,0,320.], [0,575.,240.], [0,0,1.]])
    ki = np.array([[562.,0,318.], [0,559.,242.], [0,0,1.]])
    dr = np.array([.08,-.1,.001,-.001,0.]).reshape(5,1)
    di = np.array([-.04,0,0,0,0.]).reshape(5,1)
    r = cv2.Rodrigues(np.array([.012,-.018,.007]))[0]
    t = np.array([.027,-.004,.003]).reshape(3,1)
    pairs = []
    for i in range(12):
        rv = np.array([(-.32,.15,.4)[i%3], (-.35,.25)[i%2], .08*(i%4-2)])
        rr = cv2.Rodrigues(rv)[0]
        tr = np.array([-.22+.09*(i%4), -.17+.07*(i%3), .75+.08*(i%3)]).reshape(3,1)
        rgb = fit.project(obj, rr, tr, kr, dr)
        ir = fit.project(obj, r@rr, r@tr+t, ki, di)
        pair = {'index':i+1, 'rgb_corners_px':rgb.tolist(), 'ir_corners_px':ir.tolist(),
                'rgb_stamp_ns':i*10**9, 'ir_stamp_ns':i*10**9}
        for name, matrix, distortion in [('rgb',kr,dr), ('ir',ki,di)]:
            pair[name+'_camera_info'] = {'stamp_ns':i*10**9, 'width':640, 'height':480,
                'k':matrix.reshape(-1).tolist(), 'd':distortion.reshape(-1).tolist(),
                'distortion_model':'plumb_bob'}
        pairs.append(pair)
    return {'status':'capture_finished', 'size':[640,480], 'pattern':[7,7],
            'square_m':.035, 'pairs':pairs}, kr, dr, ki, di, r, t


class OfflineFitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(1)
        cls.data, cls.kr, cls.dr, cls.ki, cls.di, cls.r, cls.t = synthetic_data()

    def test_known_stereo_recovered_without_rgb_mutation(self):
        before = self.kr.copy()
        result = fit.fit(self.data, list(range(12)), 'k1', self.kr, self.dr)
        np.testing.assert_array_equal(self.kr, before)
        np.testing.assert_allclose(result['ir_k'], self.ki, atol=.015)
        np.testing.assert_allclose(result['t_rgb_to_ir_m'], self.t.reshape(3), atol=3e-5)
        np.testing.assert_allclose(result['r_rgb_to_ir'], self.r, atol=3e-5)
        self.assertLess(result['stereo_training_rms_px'], .001)

    def test_forward_and_inverse_transform_convention(self):
        result = {'rgb_k':self.kr, 'rgb_d':self.dr, 'ir_k':self.ki, 'ir_d':self.di,
                  'r_rgb_to_ir':self.r, 't_rgb_to_ir_m':self.t.reshape(3)}
        evaluation = fit.evaluate(self.data, 3, result)
        self.assertLess(evaluation['rgb_to_ir']['rms_px'], .001)
        self.assertLess(evaluation['ir_to_rgb']['rms_px'], .001)

    def test_rgb_anchored_recovers_known_geometry(self):
        result = fit.fit_rgb_anchored(self.data, list(range(12)), 'k1', self.kr, self.dr)
        np.testing.assert_allclose(result['ir_k'], self.ki, atol=.015)
        np.testing.assert_allclose(result['t_rgb_to_ir_m'], self.t.reshape(3), atol=3e-5)
        np.testing.assert_allclose(result['r_rgb_to_ir'], self.r, atol=3e-5)
        self.assertLess(result['anchored_ir_training_rms_px'], .001)

    def test_rgb_anchored_does_not_leak_held_out_data(self):
        corrupted = copy.deepcopy(self.data)
        corrupted['pairs'][3]['ir_corners_px'] = (np.array(corrupted['pairs'][3]['ir_corners_px'])+[20,0]).tolist()
        train = [i for i in range(12) if i != 3]
        original = fit.fit_rgb_anchored(self.data, train, 'k1', self.kr, self.dr)
        excluded = fit.fit_rgb_anchored(corrupted, train, 'k1', self.kr, self.dr)
        np.testing.assert_array_equal(original['ir_k'], excluded['ir_k'])
        self.assertGreater(fit.evaluate(corrupted, 3, excluded)['rgb_to_ir']['rms_px'], 19)

    def test_held_out_view_not_used_in_fitting(self):
        corrupted = copy.deepcopy(self.data)
        corrupted['pairs'][3]['ir_corners_px'] = (np.array(corrupted['pairs'][3]['ir_corners_px'])+[20,0]).tolist()
        train = [i for i in range(12) if i != 3]
        original = fit.fit(self.data, train, 'k1', self.kr, self.dr)
        excluded = fit.fit(corrupted, train, 'k1', self.kr, self.dr)
        np.testing.assert_array_equal(original['ir_k'], excluded['ir_k'])
        self.assertGreater(fit.evaluate(corrupted, 3, excluded)['rgb_to_ir']['rms_px'], 19)

    def test_nonmonotonic_radial_mapping_rejected(self):
        self.assertTrue(fit.radial_check(self.ki, np.zeros(5))['passes'])
        self.assertFalse(fit.radial_check(self.ki, np.array([-8.,0,0,0,0]))['passes'])

    def test_metadata_and_timestamp_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'session.json'
            path.write_text(json.dumps(self.data))
            self.assertEqual(len(fit.load_dataset(path)[0]['pairs']), 12)
            bad = copy.deepcopy(self.data)
            bad['pairs'][2]['ir_camera_info']['stamp_ns'] += 1
            path.write_text(json.dumps(bad))
            with self.assertRaises(ValueError):
                fit.load_dataset(path)

    def test_output_refuses_existing_or_repository(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                fit.validate_output(directory)
        with self.assertRaises(ValueError):
            fit.validate_output(Path(fit.__file__).resolve().parents[2]/'unsafe_calibration')


if __name__ == '__main__':
    unittest.main()
