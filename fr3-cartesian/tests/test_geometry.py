import numpy as np
import pytest
from fr3_cartesian.geometry import Plane, calibrate_pivot, checked_target, rotation, rpy, tcp_from_flange, pose_to_base


def contact_poses():
    tool = np.array([18., -12., 125.])
    fixed = np.array([320., 40., 160.])
    poses = []
    for attitude in [[180, 0, 0], [150, 25, 0], [-155, -20, 30], [175, 35, -35], [145, -25, 20]]:
        poses.append([*(fixed-rotation(attitude) @ tool), *attitude])
    return tool, fixed, poses


def test_pivot_recovers_known_tcp_and_reference():
    tool, fixed, poses = contact_poses()
    result = calibrate_pivot(poses)
    np.testing.assert_allclose(result.tool[:3], tool, atol=1e-9)
    np.testing.assert_allclose(result.reference, fixed, atol=1e-9)
    assert result.maximum < 1e-9


def test_pivot_rejects_insufficient_orientation_and_bad_contact():
    _, _, poses = contact_poses()
    with pytest.raises(ValueError, match='Orientations'):
        calibrate_pivot([poses[0]]*4)
    poses[-1][0] += 5
    with pytest.raises(ValueError, match='residual'):
        calibrate_pivot(poses)


def test_tilted_plane_has_correct_handedness_and_round_trip():
    plane = Plane.teach('tilted', [[100, 200, 300], [200, 200, 300], [100, 280, 360]])
    np.testing.assert_allclose(plane.to_base([10, 20, 30]), [110, 198, 336])
    np.testing.assert_allclose(plane.from_base([110, 198, 336]), [10, 20, 30])
    assert np.linalg.det(plane.basis) == pytest.approx(1)
    pose = plane.pose()
    np.testing.assert_allclose(pose[:3], plane.origin)
    np.testing.assert_allclose(rotation(pose[3:]), plane.basis, atol=1e-10)


@pytest.mark.parametrize('points', [ [[0,0,0],[0,0,0],[0,100,0]], [[0,0,0],[100,0,0],[200,0,0]] ])
def test_plane_rejects_degenerate_points(points):
    with pytest.raises(ValueError):
        Plane.teach('bad', points)


def test_motion_preserves_attitude_and_enforces_path_limits():
    plane = Plane.teach('xy', [[0,0,0],[100,0,0],[0,100,0]])
    limits = [[-200,200],[-200,200],[0,100]]
    current = [0,0,10,180,0,0]
    assert checked_target(plane, current, [10,20,10], [180,0,0], limits) == [10,20,10,180,0,0]
    for target in ([201,0,10], [0,0,-1], [150,0,10], [float('nan'),0,10]):
        with pytest.raises(ValueError):
            checked_target(plane, current, target, [180,0,0], limits)
    with pytest.raises(ValueError, match='outside'):
        checked_target(plane, [0,0,-5,180,0,0], [0,0,10], [180,0,0], limits)
    with pytest.raises(ValueError, match='orientation changed'):
        checked_target(plane, current, [0,0,10], [170,0,0], limits)


@pytest.mark.parametrize('axis,position,minimum,maximum', [(0, 230., -200., 200.), (1, -230., -200., 200.), (2, 25., -10., 10.)])
def test_valid_origin_target_does_not_bypass_start_position_limits(axis, position, minimum, maximum):
    plane = Plane.teach('tilted', [[100, 200, 300], [200, 200, 300], [100, 280, 360]])
    start = [0., 0., 0.]
    start[axis] = position
    current = [*plane.to_base(start), 180, 0, 0]
    with pytest.raises(ValueError, match='Current TCP is outside') as error:
        checked_target(plane, current, [0, 0, 0], [180, 0, 0], [[-200,200],[-200,200],[-10,10]])
    message = error.value.message
    assert message.values == dict(axis='XYZ'[axis], position=pytest.approx(position), minimum=minimum, maximum=maximum)


@pytest.mark.parametrize('angles', [[180,0,0], [10,20,30], [30,90,40], [-20,-90,60]])
def test_rotation_roundtrip_including_gimbal_lock(angles):
    np.testing.assert_allclose(rotation(rpy(rotation(angles))), rotation(angles), atol=1e-9)


def test_tcp_includes_flange_rotation():
    np.testing.assert_allclose(tcp_from_flange([0,0,200,180,0,0], [0,0,120,0,0,0])[:3], [0,0,80], atol=1e-10)


def test_controller_wobj_observation_explains_false_origin_distance():
    old_wobj = [57.14799880981445, -356.3059997558593, 134.1380004882812,
                18.64900016784668, 2.301000356674194, -133.9720001220703]
    legacy_origin = [-707.106582115738, -450.2818209109201, -29.552013742388297]
    wrong_upload = [*legacy_origin, -162.821, 7.713, 163.700]
    observed_tcp = [-665.618, 1100.504, -267.859, -2.18, -19.878, -108.068]
    actual_base = pose_to_base(observed_tcp, wrong_upload)
    origin_base = pose_to_base([*legacy_origin, 0, 0, 0], old_wobj)
    assert np.linalg.norm(np.array(actual_base[:3])-origin_base[:3]) == pytest.approx(.665, abs=.02)
    np.testing.assert_allclose(actual_base[:3], [252.18, 447.07, -8.72], atol=.02)
    np.testing.assert_allclose(rotation(actual_base[3:]), rotation([179.930, 1.209, -89.007]), atol=2e-5)
