import unittest
import tempfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import capture_rgb_ir_calibration as capture
from capture_rgb_ir_calibration import align_ir_order, infrared_array, infrared_preview, validate_output

class PairedCaptureTests(unittest.TestCase):
    def test_private_output_paths(self):
        repo = Path(capture.__file__).resolve().parents[2]
        with self.assertRaises(ValueError):
            validate_output(repo/'private_camera_images')
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                validate_output(directory)
            self.assertEqual(validate_output(Path(directory)/'new_capture'), Path(directory).resolve()/'new_capture')

    def test_all_square_orders_with_disparity(self):
        yy, xx = np.mgrid[:7,:7]
        rgb = np.stack([xx*24+200,yy*24+100],axis=-1).astype(np.float32)
        for k in range(4):
            for flip in (False,True):
                ir = np.rot90(rgb,k,axes=(0,1))
                if flip:
                    ir = ir[::-1]
                aligned, details = align_ir_order(rgb,ir+[21,-8])
                np.testing.assert_allclose(aligned.reshape(7,7,2),rgb+[21,-8])
                self.assertEqual(details['centered_shape_rms_px'],0)

    def test_incompatible_grid_rejected(self):
        yy,xx = np.mgrid[:7,:7]
        grid = np.stack([xx*24,yy*24],axis=-1).astype(float)
        with self.assertRaises(ValueError):
            align_ir_order(grid,grid*[2,1])

    def test_ambiguous_order_rejected(self):
        grid = np.zeros((49,1,2))
        with self.assertRaises(ValueError):
            align_ir_order(grid,grid)

    def test_ir_big_endian_padded(self):
        rows = np.full((480,642),987,dtype='>u2')
        rows[:,:640] = 456
        msg = SimpleNamespace(width=640,height=480,encoding='mono16',step=1284,
                              data=rows.tobytes(),is_bigendian=True)
        result = infrared_array(msg)
        self.assertEqual(result.dtype,np.uint16)
        self.assertEqual(result.shape,(480,640))
        self.assertTrue((result==456).all())
        msg.data = msg.data[:-1]
        with self.assertRaises(ValueError):
            infrared_array(msg)

    def test_normalization(self):
        native = np.linspace(0,1023,640*480).reshape(480,640).astype(np.uint16)
        normalized, bounds = infrared_preview(native)
        self.assertEqual(normalized.dtype,np.uint8)
        self.assertEqual(normalized.min(),0)
        self.assertEqual(normalized.max(),255)
        with self.assertRaises(ValueError):
            infrared_preview(np.ones((480,640),np.uint16)*50)

if __name__ == '__main__':
    unittest.main()
