import unittest

import cv2
import numpy as np

from pcbvision.cad import BoardModel
from tools.align_pcb_cad import detect_anchor_centers, fit_similarity


class AnchorDetectionTests(unittest.TestCase):
    def test_similarity_fit_preserves_scale_rotation_and_image_axis_flip(self):
        model = np.array([[1., 15.], [33., 15.], [1., 2.], [33., 2.]])
        angle = .07
        scale = 18.
        c, s = np.cos(angle), np.sin(angle)
        matrix = scale * np.array([[c, s], [s, -c]])
        expected = model @ matrix.T + [230., 680.]

        params, fitted = fit_similarity(model, expected)

        self.assertAlmostEqual(params[0], scale, places=7)
        self.assertAlmostEqual(params[1], angle, places=7)
        self.assertTrue(np.allclose(fitted, expected, atol=1e-7))

    def test_finds_three_holes_and_predicts_occluded_fourth(self):
        holes = np.array([[1.1, 15.4], [32.7, 15.4], [1.2, 2.3], [32.8, 2.3]])
        outline = [np.array([[0., 0.], [34., 0.], [34., 21.], [0., 21.], [0., 0.]])]
        board = BoardModel(np.zeros(2), outline, holes, np.full(4, .6),
                           ["a", "b", "c", "d"], np.array([34., 21.]), "mm")
        image = np.full((1024, 1280), 170, np.uint8)
        scale = 17.8
        matrix = np.array([[scale, 0.], [0., -scale]])
        translation = np.array([220., 700.])
        expected = np.rint(holes @ matrix.T + translation).astype(int)
        for i, (x, y) in enumerate(expected):
            if i == 3:
                continue
            cv2.circle(image, (x, y), 11, 30, -1, cv2.LINE_AA)
            cv2.circle(image, (x, y), 14, 235, 2, cv2.LINE_AA)

        centers, observed, detected, weak, residuals, _ = detect_anchor_centers(image, board)

        self.assertEqual(detected, [True, True, True, False])
        self.assertEqual(weak, [False, False, False, False])
        self.assertTrue(np.all(np.linalg.norm(centers - expected[:3], axis=1) < 2.))
        self.assertIsNone(observed[3])
        self.assertLess(np.linalg.norm(centers[0] - expected[0]), 2.)
        self.assertGreater(residuals[3], 6.)
        self.assertTrue(np.all(np.isfinite(centers)))


if __name__ == "__main__":
    unittest.main()
