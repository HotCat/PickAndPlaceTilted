"""Static-array recipe geometry is deterministic and has no controller I/O."""
import pytest

from fr3_cartesian.geometry import Plane
from fr3_cartesian.tasks import build_trajectory, validate_array, validate_poses


@pytest.fixture
def plane():
    return Plane.teach('tilted', [[100, 200, 300], [200, 200, 300], [100, 300, 320]])


def test_three_by_two_grid_route_visits_destination_and_retraces_for_next_part(plane):
    points = [[0, 0, 0], [30, 0, 0], [0, 30, 0], [30, 30, 0], [0, 60, 0], [30, 60, 0]]
    destination = [500, 600, 700, 180, 0, 25]
    outbound = [450, 550, 650, 180, 0, 0], [470, 570, 680, 180, 0, 10]
    path = build_trajectory(plane, points, destination, outbound, [175, 5, 0])
    assert len(path) == 6*4 + 5*2
    assert [name for name, _ in path[:6]] == [
        'Pick 1', 'Route 1.1', 'Route 1.2', 'Place 1', 'Return 1.1', 'Return 1.2']
    assert path[0][1] == pytest.approx([*plane.to_base(points[0]), 175, 5, 0])
    assert path[1][1] == list(outbound[0])
    assert path[2][1] == list(outbound[1])
    assert path[3][1] == list(destination)
    assert path[4][1] == list(outbound[1])  # reverse order on return
    assert path[5][1] == list(outbound[0])
    assert path[6][0] == 'Pick 2'  # the next array cell follows the return route
    assert path[-1] == ('Place 6', destination)


def test_single_part_has_no_unneeded_return_segment(plane):
    path = build_trajectory(plane, [[5, 6, 7]], [300, 400, 500, 180, 0, 0],
                            [[250, 350, 450, 180, 0, 0]], [180, 0, 0])
    assert [name for name, _ in path] == ['Pick 1', 'Route 1.1', 'Place 1']


def test_independent_routes_return_along_next_parts_route(plane):
    destination = [350, 400, 500, 180, 0, 0]
    a = [180, 200, 380, 180, 0, 0]
    b = [240, 260, 390, 180, 0, 0]
    c = [300, 340, 450, 180, 0, 0]
    path = build_trajectory(plane, [[0, 0, 0], [30, 0, 0], [60, 0, 0]], destination,
                            [], [180, 0, 0], routes=[[a], [b, c], []])
    assert [name for name, _ in path] == [
        'Pick 1', 'Route 1.1', 'Place 1', 'Return 1.1', 'Return 1.2',
        'Pick 2', 'Route 2.1', 'Route 2.2', 'Place 2', 'Pick 3', 'Place 3']
    assert path[1][1] == a
    assert [pose for _, pose in path[3:5]] == [c, b]
    assert [pose for _, pose in path[6:8]] == [b, c]
    with pytest.raises(ValueError, match='own waypoint list'):
        build_trajectory(plane, [[0, 0, 0]], destination, [], [180, 0, 0], routes=[])


@pytest.mark.parametrize('rows,columns,points', [
    (0, 2, [[0, 0, 0]]*2), (11, 1, [[0, 0, 0]]*11),
    (2, 2, [[0, 0, 0]]*3), (1, 1, [[0, 0, float('nan')]]),
    (1, 1, [[2001, 0, 0]]),
])
def test_invalid_array_dimensions_cardinality_and_points_are_rejected(rows, columns, points):
    with pytest.raises(ValueError):
        validate_array(rows, columns, points)


def test_manual_routes_are_bounded_and_validate_six_axis_poses():
    with pytest.raises(ValueError, match='waypoints'):
        validate_poses([[0, 0, 0, 0, 0, 0]]*21)
    with pytest.raises(ValueError, match='coordinates'):
        validate_poses([[2001, 0, 0, 0, 0, 0]])
    with pytest.raises(ValueError):
        validate_poses([[0, 0, 0, 0, 0]])
