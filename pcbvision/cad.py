"""Read the small subset of ASCII DXF used by the H98 PCB export.

This is deliberately not a general DWG/DXF importer. Block contents are not
expanded: the selected manufacturing layer must contain the outline directly.
"""
from dataclasses import dataclass, field
from pathlib import Path
import math

import numpy as np


@dataclass
class CircleFeature:
    center: np.ndarray
    radius: float
    handle: str
    layer: str


@dataclass
class BoardModel:
    origin: np.ndarray
    outline: list[np.ndarray]
    holes: np.ndarray
    radii: np.ndarray
    hole_handles: list[str]
    size: np.ndarray
    units: str
    circle_features: list[CircleFeature] = field(default_factory=list)


def read_entities(path):
    lines = Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
    if len(lines) % 2:
        raise ValueError("Expected an ASCII DXF with code/value pairs")
    pairs = [(int(lines[i].strip()), lines[i + 1].strip())
             for i in range(0, len(lines), 2)]
    units = None
    for i, (code, value) in enumerate(pairs[:-1]):
        if code == 9 and value == "$INSUNITS":
            units = int(pairs[i + 1][1])
    if units != 4:
        raise ValueError("This prototype requires DXF $INSUNITS=4 (millimetres)")
    section = ""
    entities = []
    current = None
    for i, (code, value) in enumerate(pairs):
        if code == 0 and value == "SECTION":
            section = pairs[i + 1][1]
        if section != "ENTITIES":
            continue
        if code == 0:
            if current is not None:
                entities.append(current)
            current = None
            if value == "ENDSEC":
                break
            current = {"type": value, "pairs": []}
        elif current is not None:
            current["pairs"].append((code, value))
            current.setdefault(code, []).append(value)
    return entities


def vertices(entity):
    """Preserve per-vertex bulges, including the last-to-first segment."""
    result = []
    for code, value in entity["pairs"]:
        if code == 10:
            result.append([float(value), 0., 0.])
        elif code == 20:
            result[-1][1] = float(value)
        elif code == 42:
            result[-1][2] = float(value)
    return np.asarray(result)


def sample_polyline(entity, spacing=.12):
    v = vertices(entity)
    closed = int(entity.get(70, ["0"])[0]) & 1
    result = []
    for i in range(len(v) if closed else len(v) - 1):
        a, b = v[i, :2], v[(i + 1) % len(v), :2]
        bulge = v[i, 2]
        delta = b - a
        length = np.linalg.norm(delta)
        if length < 1e-9:
            continue
        if abs(bulge) < 1e-9:
            n = max(2, math.ceil(length / spacing))
            points = a + np.linspace(0, 1, n, endpoint=False)[:, None] * delta
        else:
            center = (a + b) / 2 + np.array([-delta[1], delta[0]]) * (1 - bulge**2) / (4 * bulge)
            radius = np.linalg.norm(a - center)
            angle = 4 * math.atan(bulge)
            n = max(3, math.ceil(abs(angle) * radius / spacing))
            theta = math.atan2(*(a - center)[::-1]) + np.linspace(0, angle, n, endpoint=False)
            points = center + radius * np.column_stack((np.cos(theta), np.sin(theta)))
        result.extend(points)
    if not closed:
        result.append(v[-1, :2])
    else:
        result.append(result[0])
    return np.asarray(result)


def sample_arc(entity, spacing=.12):
    center = np.array([float(entity[10][0]), float(entity[20][0])])
    radius = float(entity[40][0])
    start = math.radians(float(entity[50][0]))
    stop = math.radians(float(entity[51][0]))
    sweep = (stop - start) % (2 * math.pi)
    if sweep < 1e-8:
        sweep = 2 * math.pi
    count = max(3, math.ceil(sweep * radius / spacing))
    angles = start + np.linspace(0, sweep, count + 1)
    return center + radius * np.column_stack((np.cos(angles), np.sin(angles)))


def _circle_features(entities, origin, size, fixture_holes):
    circles = []
    arc_groups = {}
    for entity in entities:
        if entity["type"] == "CIRCLE":
            center = np.array([float(entity[10][0]), float(entity[20][0])])
            circles.append((center, float(entity[40][0]), entity[5][0], entity.get(8, [""])[0]))
        elif entity["type"] == "ARC":
            center = (float(entity[10][0]), float(entity[20][0]))
            radius = float(entity[40][0])
            key = (entity.get(8, [""])[0], *np.round(center, 3), round(radius, 3))
            arc_groups.setdefault(key, []).append(entity)
        elif entity["type"] == "LWPOLYLINE":
            v = vertices(entity)
            closed = int(entity.get(70, ["0"])[0]) & 1
            if closed and len(v) == 2 and np.allclose(v[:, 2], 1):
                radius = np.linalg.norm(v[1, :2] - v[0, :2]) / 2
                if radius > .05:
                    circles.append((v[:, :2].mean(axis=0), radius, entity[5][0], entity.get(8, [""])[0]))

    for group in arc_groups.values():
        sweeps = []
        for arc in group:
            start, stop = np.radians([float(arc[50][0]), float(arc[51][0])])
            sweep = (stop - start) % (2 * math.pi)
            sweeps.append(2 * math.pi if sweep < 1e-8 else sweep)
        if abs(sum(sweeps) - 2 * math.pi) < .02:
            first = group[0]
            center = np.array([float(first[10][0]), float(first[20][0])])
            circles.append((center, float(first[40][0]), first[5][0], first.get(8, [""])[0]))

    features = []
    for center, radius, handle, layer in circles:
        local = center - origin
        if np.any(local < -.1) or np.any(local > size + .1):
            continue
        if len(fixture_holes) and np.min(np.linalg.norm(fixture_holes - local, axis=1)) < .25:
            continue
        duplicate = next((item for item in features
                          if np.linalg.norm(item.center - local) < .08), None)
        if duplicate is None:
            features.append(CircleFeature(local, radius, handle, layer))
        elif radius > duplicate.radius:
            duplicate.radius = radius
            duplicate.handle = handle
            duplicate.layer = layer
    features.sort(key=lambda item: (item.center[1], item.center[0]))
    return features


def load_board(path, layer="KEEPOUTLAYER"):
    all_entities = read_entities(path)
    entities = [e for e in all_entities if e.get(8, [""])[0] == layer]
    if any(e["type"] not in ("LWPOLYLINE", "LINE", "ARC", "CIRCLE") for e in entities):
        raise ValueError("Selected layer contains unsupported entities")
    open_curves, circles, arc_groups = [], [], {}
    for e in entities:
        if e["type"] == "CIRCLE":
            center = np.array([float(e[10][0]), float(e[20][0])])
            circles.append((center, float(e[40][0]), e[5][0]))
            continue
        if e["type"] == "LINE":
            curve = np.array([[float(e[10][0]), float(e[20][0])],
                              [float(e[11][0]), float(e[21][0])]])
            ends = tuple(sorted(tuple(np.round(p, 3)) for p in curve))
            if not any(tuple(sorted(tuple(np.round(p, 3)) for p in c[[0, -1]])) == ends
                       for c in open_curves):
                open_curves.append(curve)
            continue
        if e["type"] == "ARC":
            center = (float(e[10][0]), float(e[20][0]))
            radius = float(e[40][0])
            key = (*np.round(center, 3), round(radius, 3))
            arc_groups.setdefault(key, []).append(e)
            continue
        v = vertices(e)
        closed = int(e.get(70, ["0"])[0]) & 1
        if closed:
            if len(v) == 2 and np.allclose(v[:, 2], 1):
                radius = np.linalg.norm(v[1, :2] - v[0, :2]) / 2
                # This export also contains tiny round stroke-end decorations.
                if radius > .2:
                    circles.append(((v[0, :2] + v[1, :2]) / 2, radius, e[5][0]))
            continue
        if len(v) < 2:
            continue
        curve = sample_polyline(e)
        if not any(np.allclose(curve, c, atol=1e-4) if curve.shape == c.shape else False
                   for c in open_curves):
            open_curves.append(curve)
    for group in arc_groups.values():
        sweeps = []
        for arc in group:
            start, stop = np.radians([float(arc[50][0]), float(arc[51][0])])
            sweep = (stop - start) % (2 * math.pi)
            sweeps.append(2 * math.pi if sweep < 1e-8 else sweep)
        center = np.array([float(group[0][10][0]), float(group[0][20][0])])
        radius = float(group[0][40][0])
        if abs(sum(sweeps) - 2 * math.pi) < .02:
            circles.append((center, radius, group[0][5][0]))
            continue
        for arc in group:
            open_curves.append(sample_arc(arc))
    # Connect exact endpoints. Extra drafting lines with dangling endpoints
    # are excluded; require a single closed manufacturing outline.
    curves = list(open_curves)
    loops = []
    while curves:
        loop = [curves.pop(0)]
        while np.linalg.norm(loop[-1][-1] - loop[0][0]) > 1e-3:
            end = loop[-1][-1]
            matches = [(i, rev) for i, c in enumerate(curves) for rev in [False, True]
                       if np.linalg.norm((c[-1] if rev else c[0]) - end) < 1e-3]
            if not matches:
                break
            if len(matches) != 1:
                raise ValueError("Ambiguous branching outline; choose a clean CAD layer")
            i, rev = matches[0]
            c = curves.pop(i)
            loop.append(c[::-1] if rev else c)
        if np.linalg.norm(loop[-1][-1] - loop[0][0]) < 1e-3:
            loops.append(loop)
    if len(loops) != 1:
        raise ValueError(f"Expected one closed outline; found {len(loops)}")
    outline = loops[0]
    all_points = np.concatenate(outline)
    origin = all_points.min(axis=0)
    size = np.ptp(all_points, axis=0)
    # H98 recipe: the leftmost and rightmost circle on each of two edge rows.
    # Keep the source handles in output so the operator can confirm identity.
    corner_circles = [c for c in circles if min(c[0][0] - origin[0], origin[0] + size[0] - c[0][0]) < 2]
    corner_circles.sort(key=lambda c: (-c[0][1], c[0][0]))
    if len(corner_circles) != 4:
        raise ValueError("H98 recipe requires exactly four corner circles")
    holes = np.array([c[0] - origin for c in corner_circles])
    features = _circle_features(all_entities, origin, size, holes)
    return BoardModel(origin, [c - origin for c in outline], holes,
                      np.array([c[1] for c in corner_circles]),
                      [c[2] for c in corner_circles], size, "mm", features)
