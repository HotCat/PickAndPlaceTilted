"""Check coarse pose under rotation, arbitrary missing holes, and clipped contours."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from pcbvision.cad import BoardModel
from pcbvision.registration import ContourEvidence, SIFTReference, coarse_pose, project
from tools.align_pcb_cad import detect_anchor_centers


class CoarseRegistrationTests(unittest.TestCase):
    def setUp(self):
        self.holes = np.array([[1.1, 15.4], [32.7, 15.4], [1.2, 2.3], [32.8, 2.3]])
        self.outline = np.array([[0., 0.], [3., 0.], [3., -.8], [12., -.8], [12., 1.],
                                 [20., 1.], [20., -.8], [31., -.8], [31., 0.], [34., 0.],
                                 [34., 19.], [32., 21.], [2., 21.], [0., 19.], [0., 0.]])
        self.board = BoardModel(np.zeros(2), [self.outline], self.holes, np.full(4, .6),
                                ["a", "b", "c", "d"], np.array([34., 21.]), "mm")

    def matrix(self, angle=0., center=(640., 512.)):
        angle = np.radians(angle)
        linear = 17.8 * np.array([[np.cos(angle), np.sin(angle)], [np.sin(angle), -np.cos(angle)]])
        return np.column_stack((linear, np.asarray(center) - linear @ np.array([17., 10.])))

    def render(self, matrix, texture=False):
        image = np.full((1024, 1280), 185, np.uint8)
        polygon = np.rint(project(self.outline, matrix)).astype(np.int32)
        cv2.fillPoly(image, [polygon], 65)
        cv2.polylines(image, [polygon], True, 20, 2, cv2.LINE_AA)
        if texture:
            rng = np.random.default_rng(17)
            for x, y, radius, color in zip(rng.uniform(3., 31., 250), rng.uniform(2., 19., 250),
                                           rng.integers(2, 6, 250), rng.integers(90, 240, 250)):
                point = tuple(np.rint(project([[x, y]], matrix)[0]).astype(int))
                cv2.circle(image, point, int(radius), int(color), -1)
        return image

    def candidates(self, matrix, missing=None):
        centers = project(self.holes, matrix)
        candidates = np.column_stack((centers, np.full(4, 10.7)))
        if missing is not None:
            candidates = np.delete(candidates, missing, axis=0)
        return np.vstack((candidates, [[100., 100., 10.], [1100., 850., 15.]]))

    def test_any_missing_hole_and_rotation_preserve_cad_identity(self):
        for angle, missing in ((-135, 0), (-45, 1), (43, 2), (160, 3)):
            with self.subTest(angle=angle, missing=missing):
                matrix = self.matrix(angle)
                image = self.render(matrix)
                with patch("pcbvision.registration.circle_candidates", return_value=self.candidates(matrix, missing)), \
                     patch("tools.align_pcb_cad._local_circle_center", return_value=None):
                    diagnostics = {}
                    _, observed, detected, weak, _, _ = detect_anchor_centers(image, self.board, diagnostics=diagnostics)
                self.assertFalse(detected[missing])
                self.assertIsNone(observed[missing])
                self.assertEqual(weak, [False] * 4)
                actual = np.asarray(diagnostics["cad_to_image"])
                np.testing.assert_allclose(project(self.holes, actual), project(self.holes, matrix), atol=1e-5)

    def test_contour_can_extend_beyond_image_while_three_holes_remain_visible(self):
        matrix = self.matrix(0, center=(295., 512.))
        # Board's leftmost contour is outside x=0, while all four centers are inside.
        self.assertLess(project(self.outline, matrix)[:, 0].min(), 0.)
        image = self.render(matrix)
        with patch("pcbvision.registration.circle_candidates", return_value=self.candidates(matrix)):
            fitted, _ = coarse_pose(image, self.board)
        np.testing.assert_allclose(project(self.holes, fitted), project(self.holes, matrix), atol=1e-5)

    def test_partial_contour_cost_tolerates_glared_section_and_ranks_correct_pose(self):
        matrix = self.matrix(25)
        image = self.render(matrix)
        # Occlude roughly one corner, leaving useful outline segments.
        cv2.rectangle(image, (400, 150), (660, 350), 220, -1)
        contour = ContourEvidence(image, self.board)
        wrong = matrix.copy()
        wrong[:, 2] += [35., -25.]
        correct_cost, support, _ = contour.score(matrix)
        wrong_cost, _, _ = contour.score(wrong)
        self.assertLess(correct_cost, wrong_cost)
        self.assertGreater(support, .5)

    def test_sift_reference_masks_background_and_recovers_rotation_then_roundtrips(self):
        matrix = self.matrix()
        image = self.render(matrix, texture=True)
        reference = SIFTReference.teach(image, self.board, matrix)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "reference.npz"
            reference.save(path)
            reference = SIFTReference.load(path, self.board)
        transform = cv2.getRotationMatrix2D((640., 512.), 37., .9)
        transform[:, 2] += [45., -20.]
        moved = cv2.warpAffine(image, transform, (1280, 1024), borderValue=185)
        estimated, info = reference.estimate(moved, self.board)
        expected = transform @ np.vstack((matrix, [0., 0., 1.]))
        np.testing.assert_allclose(project(self.holes, estimated), project(self.holes, expected), atol=1.5)
        self.assertGreaterEqual(info["sift_inliers"], 10)

    def test_invalid_sift_reference_falls_back_to_contour_matching(self):
        matrix = self.matrix(20)
        reference = SIFTReference.teach(self.render(matrix, texture=True), self.board, matrix)
        diagnostics = {}
        image = self.render(matrix)
        with patch("pcbvision.registration.circle_candidates", return_value=self.candidates(matrix)):
            estimated, _ = coarse_pose(image, self.board, reference, diagnostics)
        self.assertIn("sift_rejection", diagnostics)
        self.assertIn("partial contour", diagnostics["method"])
        np.testing.assert_allclose(project(self.holes, estimated), project(self.holes, matrix), atol=1e-5)

    def test_clutter_without_three_consistent_holes_is_rejected(self):
        matrix = self.matrix()
        with patch("pcbvision.registration.circle_candidates",
                   return_value=np.array([[100., 100., 12.], [110., 100., 12.], [100., 110., 12.]])):
            with self.assertRaisesRegex(ValueError, "three distinct"):
                coarse_pose(self.render(matrix), self.board)


if __name__ == "__main__":
    unittest.main()
