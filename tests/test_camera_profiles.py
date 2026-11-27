"""Profiles preserve actual SDK values and reject incompatible recipes."""
import copy
import json

import pytest

from pcbvision.camera_profiles import ProfileStore, profile_root


def parameters(preset=0):
    size = [2592, 1944] if preset == 0 else [1296, 972]
    return {'exposure': {'value': 12000., 'min': 100., 'max': 1000000.},
            'gain': {'value': 4, 'min': 1, 'max': 16},
            'gamma': {'value': 95, 'min': 0, 'max': 250},
            'contrast': {'value': 110, 'min': 0, 'max': 200},
            'auto_exposure': False, 'reverse_x': False, 'reverse_y': True,
            'resolution_id': preset,
            'resolutions': [{'id': 0, 'label': '2592 × 1944'}, {'id': 1, 'label': '1296 × 972 BIN2X2'}],
            'acquisition_geometry': {'iWidth': size[0], 'iHeight': size[1], 'binning': preset+1}}


def test_named_profiles_roundtrip_camera_scope_and_explicit_overwrite(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    assert profile_root() == tmp_path/'pickplace-workbench/camera-profiles'
    store = ProfileStore('mindvision', 'serial/123')
    data = store.save('棋盘 标定', parameters(), (2592, 1944))
    assert store.path('棋盘 标定').parent.parent.parent == profile_root()
    assert store.load('棋盘 标定') == data
    assert store.list_profiles() == (['棋盘 标定'], [])
    assert data['settings']['gamma'] == 95
    assert data['acquisition']['geometry']['iWidth'] == 2592
    assert ProfileStore('mindvision', 'other').list_profiles() == ([], [])
    assert ProfileStore('hikvision', 'serial/123').list_profiles() == ([], [])
    with pytest.raises(ValueError, match='already exists'):
        store.save('棋盘 标定', parameters(1), (1296, 972))
    store.save('棋盘 标定', parameters(1), (1296, 972), overwrite=True)
    assert store.load('棋盘 标定')['acquisition']['resolution_id'] == 1
    for name in ('', '.. /bad', 'a\\b', 'bad\nname', 'a'*81):
        with pytest.raises(ValueError):
            store.path(name)


@pytest.mark.parametrize('change', ['serial', 'backend', 'schema', 'nan', 'boolean', 'unknown'])
def test_invalid_and_incompatible_files_are_reported(tmp_path, change):
    store = ProfileStore('mindvision', '123', tmp_path)
    data = store.save('A', parameters(), (2592, 1944))
    if change in ('serial', 'backend'):
        data['camera'][change] = 'other'
    elif change == 'schema':
        data['schema_version'] = 100
    elif change == 'nan':
        data['settings']['gain'] = float('nan')
    elif change == 'boolean':
        data['settings']['reverse_x'] = 1
    else:
        data['settings']['robot_enable'] = True
    store.path('A').write_text(json.dumps(data))
    with pytest.raises(ValueError):
        store.load('A')
    names, errors = store.list_profiles()
    assert not names and len(errors) == 1


def test_preflight_ranges_presets_custom_geometry_and_readback(tmp_path):
    store = ProfileStore('mindvision', '123', tmp_path)
    data = store.save('A', parameters(), (2592, 1944))
    assert store.values_for_camera(data, parameters(1), (1296, 972))['resolution_id'] == 0
    store.verify_applied(data, parameters(), (2592, 1944))
    for key, value in [('gain', 100), ('gamma', 1.5)]:
        bad = copy.deepcopy(data)
        bad['settings'][key] = value
        with pytest.raises(ValueError):
            store.values_for_camera(bad, parameters(), (2592, 1944))
    unsupported = parameters()
    unsupported['contrast'] = None
    with pytest.raises(ValueError, match='unsupported'):
        store.values_for_camera(data, unsupported, (2592, 1944))
    changed = parameters(1)
    changed['resolutions'][0]['label'] = 'different ROI'
    with pytest.raises(ValueError, match='preset'):
        store.values_for_camera(data, changed, (1296, 972))
    changed = parameters()
    changed['gamma']['value'] = 100
    with pytest.raises(ValueError, match='gamma readback'):
        store.verify_applied(data, changed, (2592, 1944))
    auto = copy.deepcopy(data)
    auto['settings']['auto_exposure'] = True
    changed = parameters()
    changed['auto_exposure'] = True
    changed['exposure']['value'] = 24500.
    store.verify_applied(auto, changed, (2592, 1944))
    custom = parameters()
    custom['resolution_id'] = -1
    data = store.save('custom', custom, (2592, 1944))
    assert 'resolution_id' not in store.values_for_camera(data, custom, (2592, 1944))
    with pytest.raises(ValueError, match='ROI'):
        store.values_for_camera(data, parameters(1), (1296, 972))


def test_failed_atomic_save_preserves_previous_file(tmp_path, monkeypatch):
    store = ProfileStore('mindvision', '123', tmp_path)
    store.save('A', parameters(), (2592, 1944))
    before = store.path('A').read_bytes()
    def fail(*_):
        raise OSError('disk full')
    monkeypatch.setattr('launcher.config.os.replace', fail)
    with pytest.raises(OSError, match='disk full'):
        store.save('A', parameters(1), (1296, 972), overwrite=True)
    assert store.path('A').read_bytes() == before
    assert list(store.directory.glob('*.json.*')) == []
