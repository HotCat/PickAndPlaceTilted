"""Coarse PCB pose from hole geometry, partial contour evidence, and taught SIFT."""
from dataclasses import dataclass
from pathlib import Path
from itertools import combinations
import time

import cv2
import numpy as np


def pose_matrix(params):
    scale, angle, tx, ty = params
    a, b = scale * np.cos(angle), scale * np.sin(angle)
    return np.array([[a, b, tx], [b, -a, ty]], np.float64)


def project(points, matrix):
    return np.asarray(points) @ matrix[:, :2].T + matrix[:, 2]


def _outline_points(board, spacing=.3):
    points = []
    for segment in board.outline:
        for a, b in zip(segment[:-1], segment[1:]):
            count = max(1, int(np.ceil(np.linalg.norm(b - a) / spacing)))
            points.extend(a + np.linspace(0, 1, count, endpoint=False)[:, None] * (b - a))
    return np.asarray(points)


class ContourEvidence:
    """Score the visible CAD boundary without requiring a closed image contour.

    Oriented edge distance rejects background edges with unrelated directions.
    The best 65% of samples contribute to the cost, tolerating missing/glared
    contour sections. This ranks hypotheses; it is not an accuracy measurement.
    """
    def __init__(self, image, board):
        self.points = _outline_points(board)
        self.normals = np.roll(self.points, -1, axis=0) - np.roll(self.points, 1, axis=0)
        self.distance_maps = []
        gray = cv2.GaussianBlur(image, (0, 0), 1.)
        edges = cv2.Canny(gray, 40, 100)
        gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0)
        gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1)
        # Edge tangent angle (gradient rotated 90 degrees), ignoring polarity.
        angles = (np.arctan2(gy, gx) + np.pi / 2) % np.pi
        for angle in np.arange(8) * np.pi / 8:
            difference = abs((angles - angle + np.pi / 2) % np.pi - np.pi / 2)
            oriented = (edges > 0) & (difference < np.pi / 8)
            self.distance_maps.append(cv2.distanceTransform((~oriented).astype(np.uint8), cv2.DIST_L2, 3))
        self.shape = image.shape

    def score(self, matrix):
        points = project(self.points, matrix)
        x, y = np.rint(points).astype(int).T
        valid = (x >= 0) & (y >= 0) & (x < self.shape[1]) & (y < self.shape[0])
        visibility = float(valid.mean())
        if visibility < .35:
            return 20., 0., visibility
        tangents = self.normals @ matrix[:, :2].T
        bins = np.rint((np.arctan2(tangents[:, 1], tangents[:, 0]) % np.pi) * 8 / np.pi).astype(int) % 8
        selected_bins = bins[valid]
        distances = np.empty(int(valid.sum()), np.float64)
        for index, distance_map in enumerate(self.distance_maps):
            selected = selected_bins == index
            distances[selected] = distance_map[y[valid][selected], x[valid][selected]]
        keep = max(1, int(len(distances) * .65))
        cost = float(np.mean(np.sort(np.minimum(distances, 15.))[:keep]))
        support = float(np.mean(distances < 3.))
        return cost, support, visibility


@dataclass
class SIFTReference:
    points: np.ndarray
    descriptors: np.ndarray
    cad_to_reference: np.ndarray
    cad_holes: np.ndarray
    cad_size: np.ndarray

    @classmethod
    def teach(cls, image, board, matrix):
        mask = np.zeros(image.shape, np.uint8)
        polygon = project(np.concatenate(board.outline), matrix)
        cv2.fillPoly(mask, [np.rint(polygon).astype(np.int32)], 255)
        mask = cv2.erode(mask, np.ones((9, 9), np.uint8))
        keypoints, descriptors = cv2.SIFT_create(nfeatures=1400).detectAndCompute(image, mask)
        if descriptors is None or len(keypoints) < 16:
            raise ValueError("Not enough PCB texture to teach a SIFT reference")
        return cls(np.float32([k.pt for k in keypoints]), descriptors,
                   np.asarray(matrix, np.float64).copy(), board.holes.copy(), board.size.copy())

    def validate_board(self, board):
        if (self.cad_holes.shape != board.holes.shape or self.cad_size.shape != board.size.shape
                or not np.allclose(self.cad_holes, board.holes, atol=1e-4, rtol=0)
                or not np.allclose(self.cad_size, board.size, atol=1e-4, rtol=0)):
            raise ValueError("SIFT reference belongs to a different CAD board")

    def save(self, path):
        with Path(path).open("wb") as stream:
            np.savez_compressed(stream, points=self.points, descriptors=self.descriptors,
                                cad_to_reference=self.cad_to_reference,
                                cad_holes=self.cad_holes, cad_size=self.cad_size)

    @classmethod
    def load(cls, path, board):
        with np.load(path, allow_pickle=False) as data:
            result = cls(*(data[key].copy() for key in (
                "points", "descriptors", "cad_to_reference", "cad_holes", "cad_size")))
        if (result.points.ndim != 2 or result.points.shape[1] != 2
                or result.descriptors.shape != (len(result.points), 128)
                or result.cad_to_reference.shape != (2, 3)
                or not all(np.all(np.isfinite(v)) for v in (
                    result.points, result.descriptors, result.cad_to_reference))):
            raise ValueError("Invalid SIFT reference data")
        result.descriptors = result.descriptors.astype(np.float32)
        result.validate_board(board)
        return result

    def estimate(self, image, board):
        self.validate_board(board)
        factor = min(1., 960. / max(image.shape))
        search = (cv2.resize(image, (round(image.shape[1] * factor), round(image.shape[0] * factor)),
                             interpolation=cv2.INTER_AREA) if factor < 1. else image)
        keypoints, descriptors = cv2.SIFT_create(nfeatures=1600).detectAndCompute(search, None)
        if descriptors is None or len(keypoints) < 12:
            raise ValueError("Too little target image texture for SIFT")
        pairs = cv2.BFMatcher().knnMatch(self.descriptors, descriptors, k=2)
        matches = sorted((a for pair in pairs if len(pair) == 2
                          for a, b in [pair] if a.distance < .72 * b.distance), key=lambda m: m.distance)
        # Each target feature may support only one correspondence.
        unique, used = [], set()
        for match in matches:
            if match.trainIdx not in used:
                used.add(match.trainIdx)
                unique.append(match)
        if len(unique) < 12:
            raise ValueError("Fewer than 12 distinct SIFT matches")
        source = np.float32([self.points[m.queryIdx] for m in unique])
        target = np.float32([keypoints[m.trainIdx].pt for m in unique]) / factor
        transform, inliers = cv2.estimateAffinePartial2D(source, target, method=cv2.RANSAC,
                                                      ransacReprojThreshold=3., maxIters=3000)
        if transform is None or inliers is None:
            raise ValueError("SIFT RANSAC could not estimate a pose")
        inliers = inliers.ravel().astype(bool)
        count = int(inliers.sum())
        if count < 10 or count / len(unique) < .4:
            raise ValueError("SIFT match consensus is too weak")
        ref_outline = project(np.concatenate(board.outline), self.cad_to_reference)
        span = np.ptp(ref_outline, axis=0)
        coverage = np.ptp(source[inliers], axis=0) / np.maximum(span, 1)
        if np.min(coverage) < .2:
            raise ValueError("SIFT inliers occupy too small an area of the PCB")
        relative_scale = np.linalg.norm(transform[:, 0])
        if not .5 <= relative_scale <= 2.:
            raise ValueError("SIFT scale change exceeds 0.5–2 times the reference")
        matrix = transform @ np.vstack((self.cad_to_reference, [0., 0., 1.]))
        rms = float(np.sqrt(np.mean(np.sum((project(source[inliers], transform) - target[inliers])**2, axis=1))))
        return matrix, {"sift_matches": len(unique), "sift_inliers": count, "sift_rms_px": rms,
                        "sift_search_scale": factor}


def circle_candidates(image):
    gray = cv2.GaussianBlur(image, (0, 0), 1.)
    found = []
    for threshold in (26, 30):
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1., minDist=24,
                                   param1=65, param2=threshold, minRadius=8, maxRadius=20)
        if circles is not None:
            found.extend(circles[0])
    candidates = []
    for circle in found:
        near = next((item for item in candidates if np.linalg.norm(item[:2] - circle[:2]) < 3.), None)
        if near is None:
            candidates.append(np.asarray(circle, np.float64))
        else:
            near[:] = circle
    if len(candidates) < 3:
        raise ValueError("Fewer than three circular candidates; use a better exposure or a taught reference")
    return np.asarray(candidates)


def _match(matrix, board, candidates, gate=8.):
    predictions = project(board.holes, matrix)
    distances = np.linalg.norm(predictions[:, None] - candidates[None, :, :2], axis=2)
    nearest = np.argmin(distances, axis=1)
    residuals = distances[np.arange(4), nearest]
    supported = residuals < gate
    # A single image circle cannot stand in for two CAD holes.
    for candidate in set(nearest[supported]):
        indices = np.flatnonzero(supported & (nearest == candidate))
        if len(indices) > 1:
            supported[indices] = False
            supported[indices[np.argmin(residuals[indices])]] = True
    return nearest, residuals, supported


def coarse_pose(image, board, reference=None, diagnostics=None):
    """Rank all model-hole pairings against circular and partial-contour evidence."""
    started = time.perf_counter()
    candidates = circle_candidates(image)
    contour = ContourEvidence(image, board)
    diagnostics = diagnostics if diagnostics is not None else {}
    diagnostics.clear()
    if reference is not None:
        try:
            matrix, info = reference.estimate(image, board)
            _, _, supported = _match(matrix, board, candidates, gate=10.)
            cost, support, visible = contour.score(matrix)
            if supported.sum() < 3:
                raise ValueError("SIFT pose has fewer than three supporting fixture circles")
            diagnostics.update(info, method="SIFT reference + fixture circles", contour_cost_px=cost,
                               contour_support=support, contour_visible=visible,
                               coarse_ms=(time.perf_counter() - started) * 1000)
            return matrix, candidates
        except ValueError as exc:
            diagnostics["sift_rejection"] = str(exc)

    n = len(candidates)
    first, second = np.where(~np.eye(n, dtype=bool))
    vectors = candidates[second, :2] - candidates[first, :2]
    lengths = np.linalg.norm(vectors, axis=1)
    shortlist = []
    # All six CAD pairs are eligible: no fixed top row or privileged missing hole.
    for p, q in combinations(range(4), 2):
        delta = board.holes[q] - board.holes[p]
        span = np.linalg.norm(delta)
        scales = lengths / span
        angles = np.arctan2(vectors[:, 1], vectors[:, 0]) + np.arctan2(delta[1], delta[0])
        a, b = scales * np.cos(angles), scales * np.sin(angles)
        valid = (scales >= 12.) & (scales <= 24.)
        matrices = np.stack((a, b, b, -a), axis=1).reshape(-1, 2, 2)
        translations = candidates[first, :2] - np.einsum("nij,j->ni", matrices, board.holes[p])
        predicted = np.einsum("ij,nkj->nik", board.holes, matrices) + translations[:, None]
        nearest = np.empty((len(predicted), 4), np.int32)
        errors = np.empty((len(predicted), 4), np.float64)
        # Limit temporary memory when the scene contains many round pads.
        for start in range(0, len(predicted), 256):
            stop = start + 256
            distances = np.linalg.norm(predicted[start:stop, :, None] - candidates[None, None, :, :2], axis=3)
            indices = distances.argmin(axis=2)
            nearest[start:stop] = indices
            errors[start:stop] = np.take_along_axis(distances, indices[:, :, None], axis=2)[:, :, 0]
        supported = errors < 8.
        for row in range(4):
            for other in range(row):
                valid &= ~(supported[:, row] & supported[:, other] & (nearest[:, row] == nearest[:, other]))
        valid &= supported.sum(axis=1) >= 3
        geometry_cost = np.minimum(errors, 12.).sum(axis=1) + (4 - supported.sum(axis=1)) * 5.
        radii = candidates[nearest, 2]
        geometry_cost += .08 * np.sum(abs(radii - scales[:, None] * board.radii), axis=1)
        order = np.argsort(np.where(valid, geometry_cost, np.inf))[:24]
        for index in order:
            if valid[index]:
                matrix = np.column_stack((matrices[index], translations[index]))
                shortlist.append((float(geometry_cost[index]), matrix))
    if not shortlist:
        raise ValueError("No pose has three distinct supporting fixture circles")
    shortlist.sort(key=lambda item: item[0])
    unique = []
    for geometry_cost, matrix in shortlist:
        projected = project(board.holes, matrix)
        if any(np.max(np.linalg.norm(projected - prev, axis=1)) < 2. for prev in unique):
            continue
        unique.append(projected)
        cost, support, visible = contour.score(matrix)
        if visible >= .35:
            # Texture-heavy background receives a larger cost than the coherent PCB boundary.
            score = geometry_cost + 5. * cost + 6. * (1. - support)
            diagnostics.setdefault("_ranked", []).append((score, matrix, cost, support, visible))
        if len(unique) >= 48:
            break
    ranked = sorted(diagnostics.pop("_ranked", []), key=lambda item: item[0])
    if not ranked:
        raise ValueError("Too little of the PCB contour is within the image")
    _, matrix, cost, support, visible = ranked[0]
    diagnostics.update(method="fixture-circle hypotheses + partial contour", contour_cost_px=cost,
                       contour_support=support, contour_visible=visible,
                       hypotheses=len(ranked), coarse_ms=(time.perf_counter() - started) * 1000)
    return matrix, candidates
