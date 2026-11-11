"""Run with a PySide6 environment; Qt renders without opening a desktop window."""
import os
import sys
import json
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

import cv2
import numpy as np

try:
    from PySide6.QtCore import QPointF, Qt
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication
except ImportError:
    QApplication = None
else:
    from pcbvision.cad import BoardModel, CircleFeature
    from tools import select_pcb_holes as selector


@unittest.skipIf(QApplication is None, "PySide6 is not installed")
class SelectorInteractionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.qt_errors = []
        self.old_excepthook = sys.excepthook
        sys.excepthook = lambda *args: self.qt_errors.append(args)
        holes = np.array([[1., 15.], [33., 15.], [1., 2.], [33., 2.]])
        features = [CircleFeature(np.array([8. + i * 3, 8.]), .4, f"c{i}", "TOPSOLDER")
                    for i in range(6)]
        self.board = BoardModel(
            np.zeros(2), [np.array([[0., 0.], [34., 0.], [34., 21.], [0., 21.], [0., 0.]])],
            holes, np.full(4, .6), ["h1", "h2", "h3", "h4"], np.array([34., 21.]), "mm", features,
        )
        self.image = np.full((1024, 1280), 150, np.uint8)
        self.observed = holes @ np.diag([18., -18.]) + [220., 700.]
        detector = (self.observed[:3], list(self.observed), [True] * 4,
                    [False, False, False, True], [0.] * 4, 4)
        patches = [
            patch.object(selector, "load_board", return_value=self.board),
            patch.object(selector.cv2, "imread", return_value=self.image),
            patch.object(selector, "detect_anchor_centers", return_value=detector),
            patch.object(selector, "_local_circle_center", return_value=None),
        ]
        self.mocks = [p.start() for p in patches]
        for p in patches:
            self.addCleanup(p.stop)
        self.window = selector.HoleSelector("board.dxf", "camera.bmp")
        self.window.show()
        self.app.processEvents()
        self.window.canvas.fit_board()

    def tearDown(self):
        self.window.close()
        self.app.processEvents()
        sys.excepthook = self.old_excepthook
        self.assertEqual(self.qt_errors, [])

    def click_cad(self, point):
        QTest.mouseClick(self.window.canvas, Qt.LeftButton, Qt.NoModifier,
                        self.window.canvas.world_to_screen(point).toPoint())
        self.app.processEvents()

    def assign_all(self, order=(0, 1, 2, 3)):
        for slot, hole in enumerate(order):
            self.window.slot_combo.setCurrentIndex(slot)
            self.click_cad(self.board.holes[hole])

    def click_image(self, point):
        rect = self.window.image_canvas.image_rect()
        screen = QPointF(rect.left() + point[0] / 1280 * rect.width(),
                         rect.top() + point[1] / 1024 * rect.height())
        QTest.mouseClick(self.window.image_canvas, Qt.LeftButton, Qt.NoModifier, screen.toPoint())
        self.app.processEvents()

    def test_loading_and_assigning_wait_for_registration_button(self):
        detector = self.mocks[2]
        detector.assert_not_called()
        self.assertFalse(self.window.register_button.isEnabled())
        self.assign_all()
        detector.assert_not_called()
        self.assertTrue(self.window.register_button.isEnabled())
        self.window.register_button.click()
        detector.assert_called_once()
        self.assertIn("3/4 selected", self.window.recipe_label.text())
        self.assertIn("ms", self.window.runtime_label.text())
        self.window.load_image("other.bmp")
        self.window.load_path("other.dxf")
        detector.assert_called_once()
        self.assertFalse(self.window.register_button.isEnabled())
        self.assertNotIn("ms", self.window.runtime_label.text())

    def test_circle_first_then_add_button_preserves_selection_and_only_stages_pair(self):
        self.click_cad(self.board.circle_features[5].center)
        self.assertEqual(self.window.pending_feature, 5)
        self.assertIn("C6", self.window.selection_help.text())
        self.assertIn("Add probe-pad anchor", self.window.selection_help.text())
        self.window.add_extra_button.click()
        self.assertEqual(self.window.pending_feature, 5)
        self.assertIn("matching center", self.window.selection_help.text())
        self.click_image([600., 550.])
        self.assertEqual(len(self.window.extra_anchors), 1)
        self.assertIn("C6", self.window.extra_anchor_list.item(0).text())
        self.assertIsNone(self.window.pending_feature)
        self.mocks[2].assert_not_called()
        self.window.extra_anchor_list.setCurrentRow(0)
        self.window.remove_extra_button.click()
        self.assertEqual(self.window.extra_anchors, [])
        self.mocks[2].assert_not_called()

    def test_add_button_first_then_circle_stages_pair(self):
        self.window.add_extra_button.click()
        self.assertIn("Click a CAD circle", self.window.selection_help.text())
        self.click_cad(self.board.circle_features[0].center)
        self.click_image([360., 550.])
        self.assertEqual(self.window.extra_anchors[0]["feature_index"], 0)
        self.mocks[2].assert_not_called()

    def test_fit_uses_slot_mapping_and_extra_pair_when_button_clicked(self):
        self.assign_all(order=(1, 0, 3, 2))
        self.window.add_extra_button.click()
        self.click_cad(self.board.circle_features[0].center)
        self.click_image([364., 556.])
        self.mocks[2].assert_not_called()
        with patch.object(selector, "fit_similarity", wraps=selector.fit_similarity) as fit:
            self.window.register_button.click()
        fitted_cad, fitted_image = fit.call_args.args
        np.testing.assert_allclose(fitted_cad[:3], self.board.holes[[1, 0, 2]])
        np.testing.assert_allclose(fitted_image[:3], self.observed[[1, 0, 2]])
        np.testing.assert_allclose(fitted_cad[3], self.board.circle_features[0].center)
        self.assertIn("1/1 probe pairs", self.window.recipe_label.text())

    def stage_probe(self, index=0):
        self.window.add_extra_button.setChecked(True)
        self.click_cad(self.board.circle_features[index].center)
        self.click_image([364., 556.])

    def test_new_image_keeps_cad_selection_and_searches_at_new_pose(self):
        self.assign_all()
        self.stage_probe()
        self.window.load_image("moved.bmp")
        anchor = self.window.extra_anchors[0]
        self.assertEqual(anchor["feature_index"], 0)
        self.assertIsNone(anchor["clicked_image_px"])
        self.assertIsNone(anchor["image_center_px"])
        self.assertIn("awaiting detection", self.window.extra_anchor_list.item(0).text())
        self.mocks[2].assert_not_called()
        moved = self.observed + [140., -70.]
        self.mocks[2].return_value = (moved[:3], list(moved), [True] * 4,
                                     [False, False, False, True], [0.] * 4, 4)
        expected_seed = self.board.circle_features[0].center @ np.diag([18., -18.]) + [360., 630.]
        fresh_measurement = expected_seed + [1., -1.]
        self.mocks[3].return_value = fresh_measurement
        self.window.register_button.click()
        np.testing.assert_allclose(self.mocks[3].call_args.args[1], expected_seed)
        np.testing.assert_allclose(anchor["image_center_px"], fresh_measurement)
        self.assertEqual(anchor["status"], "detected")
        self.assertIn("residual", self.window.extra_anchor_list.item(0).text())

    def test_missing_probe_in_new_image_is_excluded_from_fit(self):
        self.assign_all()
        self.stage_probe()
        self.window.load_image("glare.bmp")
        with patch.object(selector, "fit_similarity", wraps=selector.fit_similarity) as fit:
            self.window.register_button.click()
        self.assertEqual(len(fit.call_args.args[0]), 3)
        self.assertIsNone(self.window.extra_anchors[0]["image_center_px"])
        self.assertIn("not found; excluded", self.window.extra_anchor_list.item(0).text())
        self.assertIn("0/1 probe pairs", self.window.recipe_label.text())

    def test_recipe_restores_slots_and_probes_without_old_image_coordinates(self):
        self.assign_all(order=(1, 0, 3, 2))
        self.stage_probe(5)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "anchors.json"
            self.window.save_anchors(path)
            payload = json.loads(path.read_text())
            self.assertEqual(payload["schema"], "pcbvision.fixture_anchors.v2")
            self.assertEqual(payload["probe_anchors"][0]["dxf_handle"], "c5")
            self.assertNotIn("clicked_image_px", payload["probe_anchors"][0])
            self.window.clear_all()
            self.window.load_anchors(path)
        self.assertEqual(self.window.canvas.assignments, {0: 1, 1: 0, 2: 3, 3: 2})
        self.assertEqual(self.window.extra_anchors[0]["feature_index"], 5)
        self.assertIsNone(self.window.extra_anchors[0]["image_center_px"])
        self.assertIn("awaiting detection", self.window.extra_anchor_list.item(0).text())
        self.mocks[2].assert_not_called()

    def test_bad_recipe_keeps_existing_selections_and_legacy_file_loads(self):
        self.assign_all()
        self.stage_probe()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "anchors.json"
            self.window.save_anchors(path)
            payload = json.loads(path.read_text())
            payload["probe_anchors"][0]["center_board_mm"] = [999., 999.]
            path.write_text(json.dumps(payload))
            with self.assertRaises(ValueError):
                self.window.load_anchors(path)
            self.assertEqual(len(self.window.extra_anchors), 1)
            self.assertEqual(len(self.window.canvas.assignments), 4)
            payload["schema"] = "pcbvision.fixture_anchors.v1"
            payload.pop("probe_anchors")
            path.write_text(json.dumps(payload))
            self.window.load_anchors(path)
        self.assertEqual(self.window.extra_anchors, [])
        self.assertEqual(len(self.window.canvas.assignments), 4)

    def test_existing_probe_can_be_repicked_without_creating_duplicate(self):
        self.stage_probe(5)
        self.click_cad(self.board.circle_features[5].center)
        self.click_image([400., 500.])
        self.assertEqual(len(self.window.extra_anchors), 1)
        np.testing.assert_allclose(self.window.extra_anchors[0]["clicked_image_px"], [400., 500.], atol=2.)

    def test_taught_sift_is_kept_for_new_images_and_only_used_on_register(self):
        self.assign_all()
        self.assertFalse(self.window.teach_reference_button.isEnabled())
        self.window.register_button.click()
        self.assertTrue(self.window.teach_reference_button.isEnabled())
        reference = type("Reference", (), {"points": np.zeros((20, 2))})()
        with patch.object(selector.SIFTReference, "teach", return_value=reference):
            self.window.teach_reference_button.click()
        self.assertIs(self.window.sift_reference, reference)
        self.assertTrue(self.window.save_reference_button.isEnabled())
        self.window.load_image("rotated.bmp")
        self.assertIs(self.window.sift_reference, reference)
        self.assertIsNone(self.window.last_registration_matrix)
        self.assertFalse(self.window.teach_reference_button.isEnabled())
        self.mocks[2].assert_called_once()
        self.window.register_button.click()
        self.assertIs(self.mocks[2].call_args.kwargs["reference"], reference)
        self.window.load_path("other.dxf")
        self.assertIsNone(self.window.sift_reference)


if __name__ == "__main__":
    unittest.main()
