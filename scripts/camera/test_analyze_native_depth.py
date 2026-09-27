import unittest
from unittest.mock import patch
import cv2
import numpy as np
from analyze_native_depth import detect_offline, rgb_to_ir_board_pose, software_register


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.k=np.array([[500.,0,320],[0,500,240],[0,0,1]])
        self.d=np.zeros(5)

    def test_identity_registration_preserves_z(self):
        depth=np.zeros((480,640),np.uint16)
        depth[240,320]=1000
        output=software_register(depth,self.k,self.d,self.k,self.d,np.eye(3),np.zeros(3))
        np.testing.assert_array_equal(output,depth)

    def test_inverse_translation_direction(self):
        depth=np.zeros((480,640),np.uint16)
        depth[240,320]=1000
        # IR = RGB + 40mm => the RGB projection is 20 pixels to the left.
        output=software_register(depth,self.k,self.d,self.k,self.d,np.eye(3),np.array([.04,0,0]))
        self.assertEqual(output[240,300],1000)
        self.assertEqual(np.count_nonzero(output),1)

    def test_out_of_view_is_not_wrapped(self):
        depth=np.zeros((480,640),np.uint16)
        depth[240,5]=1000
        output=software_register(depth,self.k,self.d,self.k,self.d,np.eye(3),np.array([.04,0,0]))
        self.assertEqual(np.count_nonzero(output),0)

    def test_board_pose_transforms_point_consistently(self):
        rr=np.array([.1,.2,.05]);tr=np.array([.03,.02,1.]).reshape(3,1)
        rotation=cv2.Rodrigues(np.array([.01,-.02,.005]))[0]
        translation=np.array([.035,0,-.004])
        ri,ti=rgb_to_ir_board_pose(rr,tr,rotation,translation)
        point=np.array([.07,.105,0]).reshape(3,1)
        expected=rotation@(cv2.Rodrigues(rr)[0]@point+tr)+translation.reshape(3,1)
        actual=cv2.Rodrigues(ri)[0]@point+ti
        np.testing.assert_allclose(actual,expected,atol=1e-12)

    def test_exhaustive_fallback_finds_complete_grid(self):
        corners=np.zeros((49,1,2),np.float32)
        with patch('analyze_native_depth.detect',return_value=None), patch('analyze_native_depth.cv2.findChessboardCornersSB',return_value=(True,corners)) as sb:
            self.assertIs(detect_offline(np.zeros((480,640),np.uint8)),corners)
            self.assertTrue(sb.call_args.kwargs['flags'] & cv2.CALIB_CB_EXHAUSTIVE)
            self.assertTrue(sb.call_args.kwargs['flags'] & cv2.CALIB_CB_ACCURACY)

    def test_partial_grid_is_rejected(self):
        with patch('analyze_native_depth.detect',return_value=None), patch('analyze_native_depth.cv2.findChessboardCornersSB',return_value=(True,np.zeros((48,1,2)))):
            self.assertIsNone(detect_offline(np.zeros((480,640),np.uint8)))

    def test_nonfinite_grid_is_rejected(self):
        with patch('analyze_native_depth.detect',return_value=None), patch('analyze_native_depth.cv2.findChessboardCornersSB',return_value=(True,np.full((49,1,2),np.nan))):
            self.assertIsNone(detect_offline(np.zeros((480,640),np.uint8)))


if __name__=='__main__':
    unittest.main()
