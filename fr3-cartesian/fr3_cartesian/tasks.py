"""Pure recipe helpers for an explicitly taught static part array.

The array stores tool-centre-point positions in a taught plane; it stores no
vision measurements or controller state. Each transit waypoint and the fixed
destination is a full TCP pose in robot base. Each part has an outbound transit
list; reversing the next part's list gives its inbound path from the destination.
"""
from .geometry import vector
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


def validate_routes(routes, count):
    """Require one independent outbound route per source in row-major order."""
    if len(routes) != count:
        raise OperatorValueError('Each array part must have its own waypoint list')
    return [validate_poses(route) for route in routes]


def build_trajectory(plane, points, destination, waypoints, source_orientation, *, routes=None):
    """Return named absolute base TCP poses in the operator's execution order.

    For every array point: approach that source, visit the manually planned
    outbound route, visit the fixed destination, then follow the NEXT source's
    outbound route in reverse to approach that next source. The final part ends
    at the destination. Legacy shared waypoints are accepted for older callers.
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
    if routes is None:
        shared = validate_poses(waypoints)
        routes = [[list(pose) for pose in shared] for _ in sources]
    transit_routes = validate_routes(routes, len(sources))
    path = []
    for index, source in enumerate(sources):
        path.append((f'Pick {index + 1}', [*plane.to_base(source), *source_attitude]))
        path.extend((f'Route {index + 1}.{point + 1}', pose) for point, pose in enumerate(transit_routes[index]))
        path.append((f'Place {index + 1}', target))
        if index + 1 < len(sources):
            path.extend((f'Return {index + 1}.{point + 1}', pose)
                        for point, pose in enumerate(reversed(transit_routes[index + 1])))
    return path
