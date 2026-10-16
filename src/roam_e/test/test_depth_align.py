import numpy as np

from roam_e.depth_align_node import shift_image


def test_shift_moves_content_and_zero_fills():
    image = np.arange(1, 13, dtype=np.uint16).reshape(3, 4)
    shifted = shift_image(image, -1, 0)
    assert shifted.tolist() == [[2, 3, 4, 0], [6, 7, 8, 0], [10, 11, 12, 0]]
    shifted = shift_image(image, 2, 1)
    assert shifted.tolist() == [[0, 0, 0, 0], [0, 0, 1, 2], [0, 0, 5, 6]]


def test_zero_shift_is_identity():
    image = np.random.default_rng(0).integers(0, 5000, (6, 8)).astype(np.uint16)
    assert np.array_equal(shift_image(image, 0, 0), image)
