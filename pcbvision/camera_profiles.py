"""Versioned, per-camera JSON profiles for camera acquisition and image tuning.

Profiles deliberately contain no robot poses or camera intrinsics. Save SDK
readback rather than unconfirmed GUI values, and validate an entire profile
before issuing any write. A backend/serial scope prevents mixing vendor scales.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import unicodedata

from launcher.config import atomic_json

SCALARS = ('exposure', 'gain', 'gamma', 'contrast')
FLAGS = ('auto_exposure', 'reverse_x', 'reverse_y')


def profile_root():
    return Path(os.environ.get('XDG_CONFIG_HOME', Path.home()/'.config'))/'pickplace-workbench/camera-profiles'


def profile_name(name):
    name = unicodedata.normalize('NFC', name.strip()) if isinstance(name, str) else ''
    if not name or len(name) > 80 or any(ord(c) < 32 or c in '/\\' for c in name):
        raise ValueError('Use a profile name of 1–80 characters without slashes / 配置名称需为1–80个字符，不能含斜杠')
    return name


def _component(text):
    # The digest distinguishes names that collapse to the same readable slug.
    slug = re.sub(r'[^\w.-]+', '-', text, flags=re.UNICODE).strip('.-')[:40] or 'profile'
    return slug+'-'+hashlib.sha256(text.encode()).hexdigest()[:16]


class ProfileStore:
    def __init__(self, backend, serial, root=None):
        if not isinstance(backend, str) or not re.fullmatch(r'[a-z][a-z0-9_-]*', backend):
            raise ValueError('Invalid camera backend')
        if not isinstance(serial, str) or not serial.strip():
            raise ValueError('Camera profiles require a camera serial / 相机缺少序列号，无法保存配置')
        self.identity = {'backend': backend, 'serial': serial}
        self.directory = Path(root if root is not None else profile_root())/backend/_component(serial)

    def path(self, name):
        return self.directory/(_component(profile_name(name))+'.json')

    def validate(self, data):
        if (not isinstance(data, dict) or data.get('schema_version') != 1 or
                data.get('kind') != 'camera_settings_profile' or data.get('camera') != self.identity):
            raise ValueError('Profile format or camera backend/serial does not match / 配置格式或相机型号接口、序列号不匹配')
        if profile_name(data.get('name')) != data['name']:
            raise ValueError('Profile name is not normalized')
        if any(not isinstance(data.get(key), str) or not data[key] for key in ('created_at', 'updated_at')):
            raise ValueError('Missing profile timestamps')
        values = data.get('settings')
        if not isinstance(values, dict) or not values or set(values)-set(SCALARS+FLAGS):
            raise ValueError('Invalid camera settings in profile')
        for key, value in values.items():
            valid = type(value) is bool if key in FLAGS else type(value) in (int, float) and math.isfinite(value)
            if not valid:
                raise ValueError(f'Invalid profile value: {key}')
        mode = data.get('acquisition')
        if (not isinstance(mode, dict) or not {'image_size', 'resolution_id', 'resolution_label', 'geometry'} <= mode.keys()
                or not isinstance(mode.get('image_size'), list) or len(mode['image_size']) != 2):
            raise ValueError('Missing acquisition mode in profile')
        if any(type(v) is not int or v <= 0 for v in mode['image_size']):
            raise ValueError('Invalid profile image dimensions')
        if mode.get('resolution_id') is not None and type(mode['resolution_id']) is not int:
            raise ValueError('Invalid profile resolution ID')
        if mode.get('geometry') is not None and not isinstance(mode['geometry'], dict):
            raise ValueError('Invalid acquisition geometry')
        if mode['geometry'] is not None and any(type(v) is not int and v is not None for v in mode['geometry'].values()):
            raise ValueError('Invalid acquisition geometry values')
        if mode['resolution_label'] is not None and not isinstance(mode['resolution_label'], str):
            raise ValueError('Invalid resolution label')
        return data

    def load(self, name):
        data = self.validate(json.loads(self.path(name).read_text(encoding='utf-8')))
        if data['name'] != profile_name(name):
            raise ValueError('Profile name does not match its file')
        return data

    def list_profiles(self):
        names, errors = [], []
        for path in sorted(self.directory.glob('*.json')):
            try:
                data = self.validate(json.loads(path.read_text(encoding='utf-8')))
                if path != self.path(data['name']):
                    raise ValueError('Profile filename does not match its name')
                names.append(data['name'])
            except Exception as exc:
                errors.append(f'{path.name}: {exc}')
        return sorted(names, key=str.casefold), errors

    def save(self, name, parameters, image_size, *, overwrite=False):
        name = profile_name(name)
        path = self.path(name)
        now = datetime.now(timezone.utc).isoformat(timespec='seconds')
        created = now
        if path.exists():
            if not overwrite:
                raise ValueError('Profile name already exists; select it and use Save / 配置已存在，请选择后点击保存')
            created = self.load(name)['created_at']
        settings = {key: parameters[key]['value'] for key in SCALARS if parameters.get(key) is not None}
        settings.update({key: bool(parameters[key]) for key in FLAGS if parameters.get(key) is not None})
        resolution_id = parameters.get('resolution_id')
        label = next((p['label'] for p in parameters.get('resolutions', []) if p['id'] == resolution_id), None)
        data = {'schema_version': 1, 'kind': 'camera_settings_profile', 'name': name,
                'camera': dict(self.identity), 'created_at': created, 'updated_at': now,
                'settings': settings, 'acquisition': {'image_size': list(image_size),
                    'resolution_id': resolution_id, 'resolution_label': label,
                    'geometry': parameters.get('acquisition_geometry')}}
        self.validate(data)
        atomic_json(path, data)
        return data

    def values_for_camera(self, data, parameters, image_size):
        """Preflight the complete recipe; never silently clamp or skip fields."""
        self.validate(data)
        values = dict(data['settings'])
        for key, value in values.items():
            capability = parameters.get(key)
            if capability is None:
                raise ValueError(f'Profile setting unsupported by this camera: {key} / 相机不支持此配置参数')
            if key in SCALARS:
                if not capability['min'] <= value <= capability['max']:
                    raise ValueError(f'Profile {key}={value} is outside the current SDK range / 配置数值超出范围')
                if self.identity['backend'] == 'mindvision' and key != 'exposure' and value != round(value):
                    raise ValueError(f'Profile {key} must use integer SDK units')
        mode = data['acquisition']
        preset_id = mode['resolution_id']
        if preset_id is not None and preset_id >= 0:
            preset = next((p for p in parameters.get('resolutions', []) if p['id'] == preset_id), None)
            if preset is None or preset['label'] != mode.get('resolution_label'):
                raise ValueError('Saved resolution preset is unavailable or changed / 保存的分辨率预设不可用或已改变')
            if preset_id != parameters.get('resolution_id'):
                values['resolution_id'] = preset_id
            elif mode['geometry'] != parameters.get('acquisition_geometry') or mode['image_size'] != list(image_size):
                raise ValueError('Current acquisition geometry differs from the saved preset / 当前采集几何与配置不一致')
        elif mode['geometry'] != parameters.get('acquisition_geometry') or mode['image_size'] != list(image_size):
            # Custom ROI and Hikvision geometry have no setter in this preview.
            raise ValueError('Restore the saved ROI/binning/resolution first; this mode cannot be changed by a profile / 请先恢复保存时的ROI、合并像素与分辨率')
        return values

    def verify_applied(self, data, parameters, image_size):
        """Do not announce success after a silent SDK coercion or partial apply."""
        mode = data['acquisition']
        if (mode['geometry'] != parameters.get('acquisition_geometry') or mode['image_size'] != list(image_size)
                or mode['resolution_id'] != parameters.get('resolution_id')):
            raise ValueError('Profile acquisition mode was not restored by the camera / 相机未恢复配置的采集模式')
        for key, value in data['settings'].items():
            if key == 'exposure' and data['settings'].get('auto_exposure'):
                continue  # Auto exposure owns this changing measurement.
            actual = parameters.get(key)
            actual = actual.get('value') if isinstance(actual, dict) else actual
            matches = actual is not None and (bool(actual) == value if key in FLAGS else
                                              math.isclose(actual, value, rel_tol=.001, abs_tol=.001))
            if not matches:
                raise ValueError(f'Profile {key} readback differs: requested {value}, camera {actual} / 相机读回数值与配置不符')
