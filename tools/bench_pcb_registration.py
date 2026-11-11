"""Compare coarse methods on saved images and controlled image transformations.

Transformed-image errors measure consistency relative to the first pose, not
physical accuracy. The reference pose used here must be visually inspected.
"""
import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2
import numpy as np

from pcbvision.cad import load_board
from pcbvision.registration import SIFTReference, project
from tools.align_pcb_cad import detect_anchor_centers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, default=Path("/home/hotcat/Downloads/uploadPCB"))
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/pcb_registration_analysis"))
    parser.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    board = load_board(args.source_dir / "H98_simple.dxf")
    paths = sorted(args.source_dir.glob("Image*.bmp"))
    if not paths:
        raise SystemExit("No Image*.bmp snapshots found")
    source = cv2.imread(str(paths[0]), 0)
    initial = {}
    detect_anchor_centers(source, board, diagnostics=initial)
    base = np.asarray(initial["cad_to_image"])
    reference = SIFTReference.teach(source, board, base)
    report = {"qualification": "Image consistency checks only; no physical ground truth or placement accuracy measured.",
              "reference_image": str(paths[0]), "reference_features": len(reference.points),
              "timing_scope": "Detection and coarse/fine hole fitting; excludes file I/O, SIFT teaching, overlays, and probe-pad refinement.",
              "saved_images": [], "transformed_source": []}
    previews = []

    def evaluate(image, ref, expected=None, repeats=1):
        times, details = [], {}
        for _ in range(max(1, repeats)):
            started = time.perf_counter()
            try:
                detect_anchor_centers(image, board, reference=ref, diagnostics=details)
            except (ValueError, cv2.error) as exc:
                return {"error": str(exc)}, None
            times.append((time.perf_counter() - started) * 1000)
        matrix = np.asarray(details["cad_to_image"])
        result = {**details, "samples_ms": times, "median_ms": float(np.median(times))}
        if expected is not None:
            result["max_hole_consistency_error_px"] = float(np.max(np.linalg.norm(
                project(board.holes, matrix) - project(board.holes, expected), axis=1)))
        return result, matrix

    for path in paths:
        image = cv2.imread(str(path), 0)
        entry = {"image": str(path)}
        for name, ref in (("holes_and_contour", None), ("sift_and_holes", reference)):
            result, matrix = evaluate(image, ref, repeats=args.repeats)
            entry[name] = result
            if matrix is not None and name == "sift_and_holes":
                preview = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                for segment in board.outline:
                    points = np.rint(project(segment, matrix)).astype(np.int32)
                    cv2.polylines(preview, [points], False, (0, 255, 0), 2, cv2.LINE_AA)
                for i, point in enumerate(project(board.holes, matrix)):
                    center = tuple(np.rint(point).astype(int))
                    cv2.drawMarker(preview, center, (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
                    cv2.putText(preview, f"H{i + 1}", (center[0] + 8, center[1] - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.imwrite(str(args.output_dir / f"{path.stem}_overlay.png"), preview)
                tile = cv2.resize(preview, (640, 512))
                cv2.putText(tile, path.stem, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .55,
                            (0, 255, 255), 1, cv2.LINE_AA)
                previews.append(tile)
        report["saved_images"].append(entry)
    center = tuple(np.mean(project(board.holes, base), axis=0))
    for angle, dx, dy in ((45, 0, 0), (90, 0, 0), (135, 0, 0), (180, 0, 0),
                           (-90, 0, 0), (0, -240, 0)):
        transform = cv2.getRotationMatrix2D(center, angle, 1.)
        transform[:, 2] += [dx, dy]
        image = cv2.warpAffine(source, transform, (source.shape[1], source.shape[0]), borderValue=150)
        expected = transform @ np.vstack((base, [0., 0., 1.]))
        entry = {"rotation_degrees": angle, "translation_px": [dx, dy]}
        for name, ref in (("holes_and_contour", None), ("sift_and_holes", reference)):
            entry[name], _ = evaluate(image, ref, expected)
        report["transformed_source"].append(entry)
    path = args.output_dir / "report.json"
    path.write_text(json.dumps(report, indent=2) + "\n")
    if previews:
        cv2.imwrite(str(args.output_dir / "overlays.png"), np.hstack(previews))
    print(json.dumps({"report": str(path), "overlays": str(args.output_dir / "overlays.png"),
                      "saved_images": [{"image": Path(e["image"]).name,
                                        "contour_ms": round(e["holes_and_contour"].get("median_ms", 0), 1),
                                        "sift_ms": round(e["sift_and_holes"].get("median_ms", 0), 1),
                                        "sift_inliers": e["sift_and_holes"].get("sift_inliers", 0)}
                                       for e in report["saved_images"]],
                      "transformed_checks": [{"angle": e["rotation_degrees"],
                                              "contour_error_px": e["holes_and_contour"].get("max_hole_consistency_error_px"),
                                              "sift_error_px": e["sift_and_holes"].get("max_hole_consistency_error_px")}
                                             for e in report["transformed_source"]]}, indent=2))


if __name__ == "__main__":
    main()
