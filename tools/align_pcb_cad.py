#!/usr/bin/env python3
"""Locate the H98 hole pattern in a PCB image and make a CAD overlay."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from pcbvision.cad import load_board


DEFAULT_IMAGE = Path("/home/hotcat/Downloads/uploadPCB/Image_20261109165458976.bmp")
DEFAULT_DXF = Path("/home/hotcat/Downloads/uploadPCB/H98_simple.dxf")


def fit_similarity(model_points, image_points):
    """Fit image = scale * rotation/reflection(CAD) + translation robustly."""
    model_points = np.asarray(model_points, np.float64)
    image_points = np.asarray(image_points, np.float64)
    if len(model_points) < 3:
        raise ValueError("At least three CAD/image anchor pairs are required")
    # x'=a*x+b*y+tx, y'=b*x-a*y+ty; solve by IRLS to reduce the
    # influence of one poorly localized hole without adding a SciPy dependency.
    rows, values = [], []
    for (x, y), (u, v) in zip(model_points, image_points):
        rows.extend(([x, y, 1., 0.], [-y, x, 0., 1.]))
        values.extend((u, v))
    design = np.asarray(rows)
    values = np.asarray(values)
    weights = np.ones(len(model_points))
    for _ in range(6):
        row_weights = np.repeat(np.sqrt(weights), 2)
        params = np.linalg.lstsq(design * row_weights[:, None],
                                 values * row_weights, rcond=None)[0]
        a, b = params[:2]
        matrix = np.array([[a, b], [b, -a]])
        predicted = model_points @ matrix.T + params[2:]
        error = np.linalg.norm(predicted - image_points, axis=1)
        weights = np.minimum(1., 1.5 / np.maximum(error, 1e-9))
    a, b = params[:2]
    scale = np.hypot(a, b)
    angle = np.arctan2(b, a)
    return np.array([scale, angle, params[2], params[3]]), predicted


def _local_circle_center(image, point, nominal_radius):
    """Find a weak local circle with repeated lower-threshold Hough votes."""
    x, y = np.round(point).astype(int)
    half = max(14, round(nominal_radius * 1.7))
    x0, x1 = max(0, x - half), min(image.shape[1], x + half + 1)
    y0, y1 = max(0, y - half), min(image.shape[0], y + half + 1)
    if x1 - x0 < 5 or y1 - y0 < 5:
        return None
    roi = cv2.GaussianBlur(image[y0:y1, x0:x1], (0, 0), 1.0)
    votes = []
    for threshold in (10, 12, 14):
        circles = cv2.HoughCircles(
            roi, cv2.HOUGH_GRADIENT, dp=1.0, minDist=10,
            param1=60, param2=threshold,
            minRadius=max(5, round(nominal_radius * .6)),
            maxRadius=max(12, round(nominal_radius * 2.0)),
        )
        if circles is None:
            continue
        candidates = circles[0]
        centers = candidates[:, :2] + [x0, y0]
        distances = np.linalg.norm(centers - point, axis=1)
        valid = ((candidates[:, 2] >= nominal_radius * .5)
                 & (candidates[:, 2] <= nominal_radius * 2.0)
                 & (distances <= 7.0))
        if np.any(valid):
            index = int(np.argmin(np.where(valid, distances, np.inf)))
            votes.append(centers[index])
    if len(votes) < 2:
        return None
    votes = np.asarray(votes)
    neighbor_counts = np.sum(np.linalg.norm(votes[:, None] - votes[None, :], axis=2) <= 2.5, axis=1)
    best = int(np.argmax(neighbor_counts))
    cluster = votes[np.linalg.norm(votes - votes[best], axis=1) <= 2.5]
    if len(cluster) < 2:
        return None
    return np.median(cluster, axis=0)


def detect_anchor_centers(image, board, reference=None, diagnostics=None):
    """Use any three visible fixture holes, with contour/SIFT coarse validation.

    Missing holes remain predictions, not fit measurements. Locally recovered
    circles are labelled weak independently of which CAD hole is obscured.
    """
    from pcbvision.registration import coarse_pose, pose_matrix, project, _match

    details = diagnostics if diagnostics is not None else {}
    matrix, candidates = coarse_pose(image, board, reference, details)
    nearest, _, supported = _match(matrix, board, candidates, gate=10.)
    indices = np.flatnonzero(supported)
    params, fitted = fit_similarity(board.holes[indices], candidates[nearest[indices], :2])
    matrix = pose_matrix(params)
    nearest, distances, supported = _match(matrix, board, candidates)
    indices = np.flatnonzero(supported)
    if len(indices) < 3:
        raise ValueError("Fewer than three fixture holes survive pose refinement")
    params, fitted = fit_similarity(board.holes[indices], candidates[nearest[indices], :2])
    matrix = pose_matrix(params)
    predicted = project(board.holes, matrix)
    observed, detected, weak, residuals = [], [], [], []
    for i, point in enumerate(predicted):
        if supported[i]:
            center = candidates[nearest[i], :2].copy()
            is_weak = False
        else:
            center = _local_circle_center(image, point, params[0] * board.radii[i])
            is_weak = center is not None
        observed.append(center)
        detected.append(center is not None)
        weak.append(is_weak)
        residuals.append(float(np.linalg.norm(center - point)) if center is not None else float(distances[i]))
    details["strong_hole_indices"] = indices.tolist()
    details["cad_to_image"] = matrix.tolist()
    details["fit_rms_px"] = float(np.sqrt(np.mean(np.sum((fitted - candidates[nearest[indices], :2])**2, axis=1))))
    # Retain the CLI's legacy three-point return value; predictions in these
    # slots must never be counted as observations by callers.
    anchors = np.array([observed[i] if supported[i] else predicted[i] for i in range(3)])
    return anchors, observed, detected, weak, residuals, len(candidates)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, default=DEFAULT_IMAGE)
    parser.add_argument("--dxf", type=Path, default=DEFAULT_DXF)
    parser.add_argument("--layer", default="KEEPOUTLAYER")
    parser.add_argument("--reference", type=Path, help="optional taught SIFT .npz reference")
    parser.add_argument("--anchors", type=float, nargs=6, default=None,
                        metavar=("X1", "Y1", "X2", "Y2", "X3", "Y3"),
                        help="optional manual image centers for CAD holes 1, 2, and 3")
    parser.add_argument("--overlay", type=Path, default=Path("pcb_cad_overlay.png"))
    parser.add_argument("--report", type=Path, default=Path("pcb_pose_estimate.json"))
    args = parser.parse_args()

    image = cv2.imread(str(args.image), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise SystemExit(f"Could not read image: {args.image}")
    board = load_board(args.dxf, args.layer)
    diagnostics = {}
    if args.anchors is None:
        from pcbvision.registration import SIFTReference
        reference = SIFTReference.load(args.reference, board) if args.reference else None
        _, observed_centers, detected, weak, hole_residuals, candidate_count = detect_anchor_centers(
            image, board, reference, diagnostics)
        fit_indices = [i for i in range(4) if detected[i] and not weak[i]]
        anchor_image = np.array([observed_centers[i] for i in fit_indices])
        params, fitted = fit_similarity(board.holes[fit_indices], anchor_image)
        anchor_source = diagnostics["method"]
    else:
        anchor_image = np.asarray(args.anchors, np.float64).reshape(3, 2)
        params, fitted = fit_similarity(board.holes[:3], anchor_image)
        observed_centers = [*anchor_image, None]
        detected = [True, True, True, False]
        weak = [False, False, False, False]
        hole_residuals = [0., 0., 0., None]
        candidate_count = None
        anchor_source = "manual image centers"
        fit_indices = [0, 1, 2]
    scale, angle, _, _ = params

    c, s = np.cos(angle), np.sin(angle)
    matrix = scale * np.array([[c, s], [s, -c]])
    translation = params[2:]
    projected_holes = board.holes @ matrix.T + translation
    projected_outline = [(segment @ matrix.T + translation).round().astype(np.int32)
                         for segment in board.outline]

    preview = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    for segment in projected_outline:
        cv2.polylines(preview, [segment], False, (20, 240, 40), 2, cv2.LINE_AA)
    for i, (point, radius) in enumerate(zip(projected_holes, board.radii)):
        center = tuple(np.round(point).astype(int))
        cv2.circle(preview, center, max(2, round(radius * scale)), (255, 160, 0), 2,
                   cv2.LINE_AA)
        cv2.drawMarker(preview, center, (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
        cv2.putText(preview, f"H{i + 1}", (center[0] + 8, center[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(args.overlay), preview)

    residuals = np.linalg.norm(fitted - anchor_image, axis=1)
    report = {
        "image": str(args.image), "dxf": str(args.dxf), "layer": args.layer,
        "units": board.units,
        "cad_board_size_mm": board.size.tolist(),
        "cad_origin": board.origin.tolist(),
        "image_pixels_per_mm": float(scale),
        "image_angle_degrees": float(np.degrees(angle)),
        "cad_to_image_matrix": matrix.tolist(),
        "cad_to_image_translation_px": translation.tolist(),
        "anchor_source": anchor_source,
        "coarse_registration": diagnostics,
        "fit_hole_indices": fit_indices,
        "fit_anchor_image_centers_px": anchor_image.tolist(),
        "fit_anchor_residuals_px": residuals.tolist(),
        "image_circle_candidate_count": candidate_count,
        "holes": [
            {"id": i + 1, "cad_handle": board.hole_handles[i],
             "cad_center_mm_from_origin": board.holes[i].tolist(),
             "cad_diameter_mm": float(2 * board.radii[i]),
             "observed_image_center_px": (observed_centers[i].tolist()
                                           if observed_centers[i] is not None else None),
             "predicted_center_px": projected_holes[i].tolist(),
             "status": ("weak local image candidate" if weak[i] else
                        "image candidate matched" if detected[i] else
                        "CAD prediction; image edge weak/obscured"),
             "candidate_residual_px": hole_residuals[i]}
            for i in range(4)
        ],
        "qualification": (
            "Exploratory CAD overlay only. Automatic Hough detections are geometrically "
            "matched to the known four-hole layout; weak centers may be CAD predictions. "
            "No camera calibration, independent ground truth, "
            "or production repeatability test is included. Do not use for motion control."
        ),
    }
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"overlay": str(args.overlay), "report": str(args.report),
                      "scale_px_per_mm": scale, "angle_degrees": np.degrees(angle),
                      "anchor_residuals_px": residuals.tolist(),
                      "matched_image_centers_px": anchor_image.tolist(),
                      "observed_image_centers_px": [p.tolist() if p is not None else None
                                                     for p in observed_centers],
                      "detected": detected, "candidate_residuals_px": hole_residuals,
                      "predicted_holes_px": projected_holes.tolist()}, indent=2))


if __name__ == "__main__":
    main()
