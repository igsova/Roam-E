"""Learned local features for the R19 visual relocalizer (XFeat or SuperPoint).

XFeat (verlab/accelerated_features) is the default: ~0.6 s per 640x480
frame on the Pi 5 CPU versus ~6 s for SuperPoint, and it matched our indoor
frames better (16 Oct 2026, docs/research/CPU_VISUAL_SLAM_2026-10.md).
Both offer a fast mutual-nearest-neighbour match for retrieval and a learned
LightGlue-family matcher for the final pose.
"""
import os
import sys

import numpy as np

DEFAULT_XFEAT_DIR = os.path.expanduser('~/venvs/src/accelerated_features')


class VisualFeatures:
    def __init__(self, kind='xfeat', max_keypoints=None, threads=None, xfeat_dir=None):
        import torch
        torch.set_grad_enabled(False)
        if threads:
            torch.set_num_threads(int(threads))
        self.kind = kind
        self._torch = torch
        if kind == 'xfeat':
            sys.path.insert(0, xfeat_dir or os.environ.get('XFEAT_DIR', DEFAULT_XFEAT_DIR))
            from modules.xfeat import XFeat
            self._top_k = int(max_keypoints or 1024)
            self._xf = XFeat(top_k=self._top_k)
        elif kind == 'superpoint':
            from lightglue import LightGlue, SuperPoint
            self._sp = SuperPoint(max_num_keypoints=int(max_keypoints or 512)).eval()
            self._lg = LightGlue(features='superpoint').eval()
        else:
            raise ValueError(f'unknown feature type {kind!r}')

    def extract(self, grey):
        """grey: (H, W) uint8. Returns keypoints (N,2), descriptors (N,D) as
        torch tensors and the image size (w, h)."""
        torch = self._torch
        h, w = grey.shape
        if self.kind == 'xfeat':
            rgb = np.repeat(grey[..., None], 3, axis=2)
            out = self._xf.detectAndCompute(self._xf.parse_input(rgb), top_k=self._top_k)[0]
            return {'keypoints': out['keypoints'], 'descriptors': out['descriptors'],
                    'image_size': (w, h)}
        out = self._sp.extract(torch.from_numpy(grey).float()[None, None] / 255.0)
        return {'keypoints': out['keypoints'][0], 'descriptors': out['descriptors'][0],
                'image_size': (w, h)}

    def match_fast(self, a, b, min_cossim=0.82):
        """Mutual nearest neighbours on descriptors. Returns (M, 2) indices."""
        sim = a['descriptors'] @ b['descriptors'].T
        ab = sim.argmax(1)
        ba = sim.argmax(0)
        idx0 = self._torch.arange(len(ab))
        keep = (ba[ab] == idx0) & (sim[idx0, ab] > min_cossim)
        return np.stack([idx0[keep].numpy(), ab[keep].numpy()], axis=1)

    def match_fine(self, a, b):
        """Learned matcher (LighterGlue for XFeat, LightGlue for SuperPoint)."""
        torch = self._torch
        if self.kind == 'xfeat':
            _, _, idx = self._xf.match_lighterglue(a, b)
            return np.asarray(idx).reshape(-1, 2)
        size = torch.tensor([list(a['image_size'])], dtype=torch.float32)
        out = self._lg({
            'image0': {'keypoints': a['keypoints'][None], 'descriptors': a['descriptors'][None],
                       'image_size': size},
            'image1': {'keypoints': b['keypoints'][None], 'descriptors': b['descriptors'][None],
                       'image_size': size}})
        return out['matches'][0].numpy().reshape(-1, 2)
