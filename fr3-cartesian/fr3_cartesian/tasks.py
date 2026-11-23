"""Pure recipe helpers for an explicitly taught static part array.

The array stores tool-centre-point positions in a taught plane; it stores no
vision measurements or controller state. Each transit waypoint and the fixed
destination is a full TCP pose in robot base. Reversing the transit list gives
the deliberately simple return path from the place point to the next pick point.
"""
from .geometry import Plane, vector
from .messages import OperatorValueError


def validate_array(rows, columns, points):
    """Validate a bounded rectangular list of finite plane-coordinate points."""
    if isinstance(rows, bool) or isinstance(columns, bool) or not 1 <= rows <= 10 or not 1 <= columns <= 10:
        raise OperatorValueError('Array dimensions must be between 1 and 10')
    if len(points) != rows * columns:
        raise OperatorValueError('Array must contain one taught point for every row and column')
    result = [vector(point, 3).tolist() for point in points]
    if any(abs(value) > 2000 for point in result for value in point):
        raise OperatorValueError('Plane XYZ coordinates must be between −2000 and 2000 mm')
    return result


def validate_poses(poses, maximum=20):
    """Validate manually taught six-axis base TCP poses; lengths are mm/degrees."""
    if len(poses) > maximum:
        raise OperatorValueError('Too many manual route waypoints')
    result = [vector(pose, 6).tolist() for pose in poses]
    if any(abs(value) > 2000 for pose in result for value in pose[:3]):
        raise OperatorValueError('Base TCP coordinates must be between −2000 and 2000 mm')
    return result


def build_trajectory(plane, points, destination, waypoints, source_orientation):
    """Return named absolute base TCP poses in the operator's execution order.

    For every array point: approach that source, visit the manually planned
    outbound route, visit the fixed destination, retrace the route in reverse,
    then approach the next source. The final part ends at the destination.
    Source positions inherit the caller's locked TCP orientation.
    """
    sources = [vector(p, 3).tolist() for p in points]
    if any(abs(value) > 2000 for point in sources for value in point):
        raise OperatorValueError('Plane XYZ coordinates must be between −2000 and 2000 mm')
    if not sources:
        raise OperatorValueError('Teach at least one workpiece point')
    target = vector(destination, 6).tolist()
    if any(abs(value) > 2000 for value in target[:3]):
        raise OperatorValueError('Base TCP coordinates must be between −2000 and 2000 mm')
    source_attitude = vector(source_orientation, 3).tolist()
    transit = validate_poses(waypoints)
    path = []
    for index, source in enumerate(sources):
        path.append((f'Pick {index + 1}', [*plane.to_base(source), *source_attitude]))
        path.extend((f'Route {index + 1}.{point + 1}', pose) for point, pose in enumerate(transit))
        path.append((f'Place {index + 1}', target))
        if index + 1 < len(sources):
            path.extend((f'Return {index + 1}.{point + 1}', pose)
                        for point, pose in enumerate(reversed(transit)))
    return path
