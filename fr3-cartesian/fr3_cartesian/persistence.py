"""Validated calibration documents and atomic local storage, without robot I/O."""
from dataclasses import asdict
import json
import math
from pathlib import Path

from PySide6.QtCore import QIODevice, QSaveFile

from .geometry import Plane, calibrate_pivot, vector
from .messages import OperatorValueError
from .tasks import validate_array, validate_poses


def bounded(value, low, high):
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise OperatorValueError('Invalid saved calibration setting')
    return value


def integer(value, low, high):
    if isinstance(value, bool):
        raise OperatorValueError('Invalid saved calibration setting')
    result = bounded(value, low, high)
    if result != int(result):
        raise OperatorValueError('Invalid saved calibration setting')
    return int(result)


def decode_calibration(data):
    """Normalize legacy exports and session files before changing any UI state."""
    if not isinstance(data, dict) or data.get('schema') not in (1, 2, 3):
        raise OperatorValueError('Unsupported calibration file')
    source = data.get('source')
    if source not in (None, 'simulation', 'robot'):
        raise OperatorValueError('Unsupported calibration file')
    samples = [vector(p, 6).tolist() for p in data['samples']]
    tolerance = bounded(data['tolerance'], .05, 2)
    # Derived results are always recalculated from the original measurements.
    result = asdict(calibrate_pivot(samples, tolerance)) if data.get('tool') is not None else None
    legacy_reference = 'legacy_workpiece_unknown' if source == 'robot' else 'robot_base'
    def reference(value):
        if value not in ('robot_base', 'legacy_workpiece_unknown'):
            raise OperatorValueError('Unknown calibration coordinate reference')
        return value
    samples_reference = reference(data.get('samples_reference', legacy_reference))
    planes = [Plane.teach(p['name'], p['points'], reference(p.get('reference', legacy_reference))) for p in data['planes']]
    if len({p.name for p in planes}) != len(planes):
        raise OperatorValueError('Choose a unique plane name')
    if ((samples or planes) and source is None) or (planes and result is None):
        raise OperatorValueError('Invalid saved calibration setting')
    bounds = [vector(pair, 2).tolist() for pair in data['bounds']]
    if len(bounds) != 3 or any(not -2000 <= low < high <= 2000 for low, high in bounds):
        raise OperatorValueError('Invalid travel limits')
    draft = data.get('plane_draft', {})
    points = [vector(p, 3).tolist() for p in draft.get('points', [])]
    if len(points) > 3 or points and (result is None or source is None):
        raise OperatorValueError('Invalid saved calibration setting')
    name = draft.get('name')
    if name is not None and not isinstance(name, str):
        raise OperatorValueError('Invalid saved calibration setting')
    selected = data.get('selected_plane', planes[0].name if planes else None)
    if selected is not None and selected not in {p.name for p in planes}:
        raise OperatorValueError('Invalid saved calibration setting')
    xyz = data.get('xyz', {})
    targets = vector(xyz.get('targets', [0, 0, 0]), 3).tolist()
    if any(abs(v) > 2000 for v in targets):
        raise OperatorValueError('Invalid saved calibration setting')
    orientation = xyz.get('orientation_reference')
    if orientation is not None:
        orientation = vector(orientation, 3).tolist()
    hold = xyz.get('hold', False)
    if not isinstance(hold, bool):
        raise OperatorValueError('Invalid saved calibration setting')
    jog = data.get('jog', {})
    task = data.get('task', {})
    array_rows = integer(task.get('rows', 3), 1, 10)
    array_columns = integer(task.get('columns', 2), 1, 10)
    default_points = [[column * 30., row * 30., 0.]
                      for row in range(array_rows) for column in range(array_columns)]
    array_points = validate_array(array_rows, array_columns, task.get('points', default_points))
    destination = vector(task.get('destination', [300., 0., 200., 180., 0., 0.]), 6).tolist()
    if any(abs(value) > 2000 for value in destination[:3]):
        raise OperatorValueError('Invalid saved calibration setting')
    saved_tab = integer(data.get('active_tab', 0), 0, 4)
    # Before the pick/place tab existed, saved index 3 was the retired joystick
    # pendant. Reopen those calibration sessions on the useful XYZ panel.
    if 'task' not in data and saved_tab == 3:
        saved_tab = 2
    return dict(
        source=source, samples=samples, samples_reference=samples_reference,
        tolerance=tolerance, tool=result, planes=planes,
        slot=integer(data['slot'], 1, 14), tool_id=integer(data['tool_id'], 0, 14),
        load=integer(data['load'], 0, 99), bounds=bounds,
        workpiece_slot=integer(data.get('workpiece_slot', 1), 1, 14),
        plane_draft=dict(name=name, points=points, reference=reference(draft.get('reference', legacy_reference))), selected_plane=selected,
        xyz=dict(targets=targets, step=bounded(xyz.get('step', 1), .1, 10),
                 speed=bounded(xyz.get('speed', 3), .5, 10), hold=hold,
                 orientation_reference=orientation),
        active_tab=saved_tab,
        jog=dict(joint_speed=integer(jog.get('joint_speed', 3), 1, 10),
                 tcp_speed=integer(jog.get('tcp_speed', 3), 1, 10),
                 joint_limit=bounded(jog.get('joint_limit', .5), .1, 10),
                 linear_limit=bounded(jog.get('linear_limit', 5), .1, 50),
                 rotation_limit=bounded(jog.get('rotation_limit', 1), .1, 10)),
        task=dict(rows=array_rows, columns=array_columns, points=array_points,
                  destination=destination,
                  waypoints=validate_poses(task.get('waypoints', []))),
    )


def read_calibration(path):
    return decode_calibration(json.loads(Path(path).read_text(encoding='utf-8')))


def write_calibration(path, data):
    encoded = json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False).encode('utf-8')
    output = QSaveFile(str(path))
    if not output.open(QIODevice.WriteOnly):
        raise OSError(output.errorString())
    if output.write(encoded) != len(encoded):
        output.cancelWriting()
        raise OSError(output.errorString())
    if not output.commit():
        raise OSError(output.errorString())
