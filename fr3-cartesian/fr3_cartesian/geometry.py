"""All poses use millimetres and degrees; R = Rz(rz) Ry(ry) Rx(rx)."""
from dataclasses import dataclass
import math
import numpy as np
from .messages import OperatorValueError


def vector(value, size):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise OperatorValueError("Expected {size} finite numbers", size=size)
    return a


def rotation(rpy):
    x, y, z = np.radians(vector(rpy, 3))
    cx, cy, cz = np.cos([x, y, z])
    sx, sy, sz = np.sin([x, y, z])
    return np.array([[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                     [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
                     [-sy, cy*sx, cy*cx]])


def rpy(matrix):
    y = math.asin(float(np.clip(-matrix[2, 0], -1, 1)))
    if abs(math.cos(y)) > 1e-8:
        x, z = math.atan2(matrix[2, 1], matrix[2, 2]), math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        x, z = 0.0, math.atan2(-matrix[0, 1], matrix[1, 1])
    return np.degrees([x, y, z]).tolist()


def tcp_from_flange(flange, tool):
    f, t = vector(flange, 6), vector(tool, 6)
    return np.r_[f[:3] + rotation(f[3:]) @ t[:3],
                 rpy(rotation(f[3:]) @ rotation(t[3:]))].tolist()


def pose_to_base(pose_in_workpiece, workpiece_in_base):
    """Compose base←workpiece and workpiece←pose, including orientation."""
    p, w = vector(pose_in_workpiece, 6), vector(workpiece_in_base, 6)
    return np.r_[w[:3] + rotation(w[3:]) @ p[:3],
                 rpy(rotation(w[3:]) @ rotation(p[3:]))].tolist()


@dataclass
class PivotResult:
    tool: list
    reference: list
    rms: float
    maximum: float
    condition: float


def calibrate_pivot(poses, tolerance=0.5):
    """Fit R_i * TCP + flange_i = fixed reference. Tool axes stay flange-aligned."""
    if len(poses) < 4:
        raise OperatorValueError("Capture at least four orientations touching the same fixed point")
    samples = [vector(p, 6) for p in poses]
    a = np.vstack([np.c_[rotation(p[3:]), -np.eye(3)] for p in samples])
    b = -np.concatenate([p[:3] for p in samples])
    solution, _, rank, singular = np.linalg.lstsq(a, b, rcond=None)
    condition = float(singular[0] / max(singular[-1], 1e-15))
    if rank < 6 or condition > 100:
        raise OperatorValueError("Orientations do not constrain the TCP. Tilt around two different axes and recapture")
    errors = (a @ solution - b).reshape(-1, 3)
    distances = np.linalg.norm(errors, axis=1)
    rms, maximum = float(np.sqrt(np.mean(distances**2))), float(max(distances))
    if maximum > tolerance:
        raise OperatorValueError("Contact residual {maximum:.3f} mm exceeds {tolerance:.3f} mm; recapture the points", maximum=maximum, tolerance=tolerance)
    return PivotResult([*solution[:3].tolist(), 0., 0., 0.], solution[3:].tolist(), rms, maximum, condition)


@dataclass
class Plane:
    name: str
    origin: list
    basis: list  # columns are plane +X, +Y, +Z in robot base
    points: list
    reference: str = 'robot_base'

    @classmethod
    def teach(cls, name, points, reference='robot_base'):
        if reference not in ('robot_base', 'legacy_workpiece_unknown'):
            raise OperatorValueError("Unknown calibration coordinate reference")
        if not name.strip():
            raise OperatorValueError("Give the plane a name")
        if len(points) != 3:
            raise OperatorValueError("Capture origin, a +X point, then a point on the +Y side of the XY plane")
        o, px, py = [vector(p, 3) for p in points]
        x, y = px-o, py-o
        if min(np.linalg.norm(x), np.linalg.norm(y)) < 10:
            raise OperatorValueError("Reference points must be at least 10 mm from the origin")
        x /= np.linalg.norm(x)
        z = np.cross(x, y)
        if np.linalg.norm(z) / np.linalg.norm(y) < 0.1:
            raise OperatorValueError("Plane points are nearly collinear; move the third point farther toward +Y")
        z /= np.linalg.norm(z)
        y = np.cross(z, x)
        return cls(name.strip(), o.tolist(), np.column_stack([x, y, z]).tolist(),
                   [vector(p, 3).tolist() for p in points], reference)

    def to_base(self, xyz):
        return (vector(self.origin, 3) + np.asarray(self.basis) @ vector(xyz, 3)).tolist()

    def from_base(self, xyz):
        return (np.asarray(self.basis).T @ (vector(xyz, 3)-vector(self.origin, 3))).tolist()

    def pose(self):
        return [*self.origin, *rpy(np.asarray(self.basis))]


def perpendicular_tool_orientation(plane, current_orientation):
    """Orient TCP +Z normal to a taught plane while minimizing tool-roll change.

    The normal sign is selected to be closest to the current TCP +Z direction,
    avoiding an unnecessary 180-degree flip. Current TCP +X is projected onto
    the plane to preserve roll; a plane-axis fallback handles near-parallel
    configurations where that projection has almost zero length.
    """
    if plane.reference != 'robot_base':
        raise OperatorValueError("This plane was captured with the old coordinate-frame bug. Load a recovered base-frame calibration or reteach the plane before using it.")
    basis = np.asarray(plane.basis, dtype=float)
    current = rotation(current_orientation)
    normal = basis[:, 2].copy()
    if float(np.dot(normal, current[:, 2])) < 0:
        normal *= -1

    def project(axis):
        value = axis-normal*float(np.dot(normal, axis))
        length = float(np.linalg.norm(value))
        return value/length if length > 1e-8 else None

    x_axis = project(current[:, 0])
    if x_axis is None:
        x_axis = project(current[:, 1])
    if x_axis is None:
        x_axis = project(basis[:, 0])
    if x_axis is None:
        raise OperatorValueError("Cannot determine a stable tool roll for this plane")
    y_axis = np.cross(normal, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, normal)
    return rpy(np.column_stack([x_axis, y_axis, normal]))


def check_current_plane_position(plane, current_base, bounds):
    if plane.reference != 'robot_base':
        raise OperatorValueError("This plane was captured with the old coordinate-frame bug. Load a recovered base-frame calibration or reteach the plane before using it.")
    limits = np.asarray(bounds, dtype=float)
    current = vector(current_base, 6)
    if limits.shape != (3, 2) or not np.isfinite(limits).all() or np.any(limits[:, 0] >= limits[:, 1]):
        raise OperatorValueError("Each minimum must be smaller than its maximum")
    start = np.array(plane.from_base(current[:3]))
    outside = np.flatnonzero((start < limits[:, 0]-1e-6) | (start > limits[:, 1]+1e-6))
    if len(outside):
        axis = int(outside[0])
        raise OperatorValueError("Current TCP is outside plane limits: {axis}={position:.3f} mm; allowed [{minimum:.3f}, {maximum:.3f}] mm. Reposition using teaching controls.",
                                 axis='XYZ'[axis], position=float(start[axis]),
                                 minimum=float(limits[axis, 0]), maximum=float(limits[axis, 1]))
    return start


def checked_target(plane, current_base, target, orientation, bounds, max_distance=100):
    xyz = vector(target, 3)
    current = vector(current_base, 6)
    check_current_plane_position(plane, current, bounds)
    limits = np.asarray(bounds, dtype=float)
    if np.any(xyz < limits[:, 0]) or np.any(xyz > limits[:, 1]):
        raise OperatorValueError("Target is outside the plane limits")
    result = [*plane.to_base(xyz), *vector(orientation, 3).tolist()]
    if np.linalg.norm(np.array(result[:3])-current[:3]) > max_distance:
        raise OperatorValueError("Single move exceeds {max_distance:g} mm; use smaller moves", max_distance=max_distance)
    difference = rotation(current[3:]).T @ rotation(orientation)
    angle = math.degrees(math.acos(float(np.clip((np.trace(difference)-1)/2, -1, 1))))
    if angle > 1:
        raise OperatorValueError("Tool orientation changed; lock the current orientation again")
    return result


def check_pick_point(target, bounds):
    """Apply the workpiece pickup box to a source point, not its transfer route."""
    xyz = vector(target, 3)
    limits = np.asarray(bounds, dtype=float)
    if limits.shape != (3, 2) or not np.isfinite(limits).all() or np.any(limits[:, 0] >= limits[:, 1]):
        raise OperatorValueError("Each minimum must be smaller than its maximum")
    outside = np.flatnonzero((xyz < limits[:, 0]-1e-6) | (xyz > limits[:, 1]+1e-6))
    if len(outside):
        axis = int(outside[0])
        raise OperatorValueError('Pick point outside plane limits: {axis}={position:.3f} mm; allowed [{minimum:.3f}, {maximum:.3f}] mm',
                                 axis='XYZ'[axis], position=float(xyz[axis]),
                                 minimum=float(limits[axis, 0]), maximum=float(limits[axis, 1]))


def checked_sequence_target(current_base, target_pose, max_distance=100):
    """Validate one base-frame recipe pose without constraining it to the pick plane.

    Transit and destination poses intentionally travel outside the workpiece
    pickup box and may use independently captured orientations. Keep every
    absolute base TCP pose bounded and every straight segment short enough to
    review and execute as an individual MoveL.
    """
    current = vector(current_base, 6)
    target = vector(target_pose, 6)
    if any(abs(value) > 2000 for value in target[:3]):
        raise OperatorValueError('Base TCP coordinates must be between −2000 and 2000 mm')
    distance = float(np.linalg.norm(target[:3]-current[:3]))
    if distance > max_distance:
        raise OperatorValueError('Sequence segment {distance:.1f} mm exceeds the {maximum:g} mm cap; add an intermediate waypoint or reposition before starting',
                                 distance=distance, maximum=max_distance)
    return target.tolist()
