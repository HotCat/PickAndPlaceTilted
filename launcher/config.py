"""Versioned tool catalogue and per-user settings; argv is never run in a shell."""
from __future__ import annotations

import copy
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import shutil
import tempfile

REPOSITORY = Path(__file__).resolve().parents[1]
IDENTIFIER = re.compile(r'^[a-zA-Z_][a-zA-Z0-9_]*$')
KINDS = {'string', 'path', 'ipv4', 'choice', 'boolean', 'integer'}


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def config_path():
    return Path(os.environ.get('XDG_CONFIG_HOME', Path.home()/'.config'))/'pickplace-launcher/config.json'


def state_path():
    return Path(os.environ.get('XDG_STATE_HOME', Path.home()/'.local/state'))/'pickplace-launcher'


def default_config(repository=REPOSITORY):
    data = json.loads(Path(__file__).with_name('defaults.json').read_text())
    # Reuse the working robot interpreter, vendor SDK and taught calibration on
    # this workstation, but import source from the repository. Do not copy or
    # fork calibration files merely because the launch entry point changed.
    legacy = repository.parent
    robot_home = legacy if (legacy/'.venv/bin/python').exists() else repository/'fr3-cartesian'
    replacements = {'{robot_home}': str(robot_home), '{repo}': str(repository)}
    for tool in data['tools']:
        for parameter in tool['parameters']:
            if isinstance(parameter.get('default'), str):
                for token, value in replacements.items():
                    parameter['default'] = parameter['default'].replace(token, value)
    return data


def parameter_value(tool, definition):
    return tool.get('values', {}).get(definition['key'], definition.get('default', ''))


def validate_value(definition, value):
    label, kind = definition['label'], definition['type']
    if kind == 'boolean':
        valid = type(value) is bool
    elif kind == 'integer':
        valid = type(value) is int and definition.get('min', -2147483648) <= value <= definition.get('max', 2147483647)
    else:
        valid = isinstance(value, str) and '\0' not in value and (not definition.get('required', True) or bool(value.strip()))
        if valid and kind == 'choice':
            valid = value in definition['choices']
        if valid and kind == 'ipv4':
            try:
                ipaddress.IPv4Address(value)
            except ipaddress.AddressValueError:
                valid = False
    if not valid:
        raise ValueError(f'Invalid value / 参数无效: {label}')
    return value


def validate_config(data):
    if not isinstance(data, dict) or data.get('schema_version') != 1 or not isinstance(data.get('tools'), list):
        raise ValueError('Unsupported launcher configuration / 不支持的配置格式')
    ids = set()
    for tool in data['tools']:
        if not isinstance(tool, dict):
            raise ValueError('Tool entries must be objects / 程序定义必须是对象')
        identity = tool.get('id', '')
        if not isinstance(identity, str) or not re.fullmatch(r'[a-zA-Z0-9_.-]+', identity) or identity in ids:
            raise ValueError('Tool IDs must be unique / 程序 ID 必须唯一')
        ids.add(identity)
        if not isinstance(tool.get('name'), str) or not tool['name'].strip():
            raise ValueError(f'{identity}: name is required')
        for key in ('command', 'python_paths'):
            values = tool.get(key, [])
            if not isinstance(values, list) or (key == 'command' and not values) or any(not isinstance(v, str) or not v or '\0' in v for v in values):
                raise ValueError(f'{identity}: {key} must be an argument/path list')
        if not isinstance(tool.get('cwd'), str) or not tool['cwd']:
            raise ValueError(f'{identity}: cwd is required')
        if not isinstance(tool.get('instance_key', identity), str) or not tool.get('instance_key', identity):
            raise ValueError(f'{identity}: invalid instance_key')
        if tool.get('exclusive_module') is not None and (not isinstance(tool['exclusive_module'], str) or not re.fullmatch(r'[a-zA-Z0-9_.]+', tool['exclusive_module'])):
            raise ValueError(f'{identity}: invalid exclusive_module')
        if not isinstance(tool.get('parameters', []), list) or not isinstance(tool.get('values', {}), dict):
            raise ValueError(f'{identity}: invalid parameter definitions')
        keys = set()
        for definition in tool.get('parameters', []):
            if not isinstance(definition, dict):
                raise ValueError(f'{identity}: parameter definitions must be objects')
            key = definition.get('key', '')
            if not isinstance(key, str) or not IDENTIFIER.fullmatch(key) or key in keys or definition.get('type') not in KINDS or not definition.get('label'):
                raise ValueError(f'{identity}: invalid parameter {key}')
            keys.add(key)
            if definition['type'] == 'choice':
                choices = definition.get('choices')
                if not isinstance(choices, list) or not choices or any(not isinstance(c, str) or not c for c in choices):
                    raise ValueError(f'{identity}: {key} needs choices')
            if definition.get('env') and not IDENTIFIER.fullmatch(definition['env']):
                raise ValueError(f'{identity}: invalid environment binding')
            if definition['type'] == 'integer':
                low, high = definition.get('min', -2147483648), definition.get('max', 2147483647)
                if type(low) is not int or type(high) is not int or not -2147483648 <= low <= high <= 2147483647:
                    raise ValueError(f'{identity}: invalid integer range')
            validate_value(definition, parameter_value(tool, definition))
        if set(tool.get('values', {})) - keys:
            raise ValueError(f'{identity}: values contain unknown parameters')
        discovery = tool.get('device_discovery')
        if discovery is not None:
            if (not isinstance(discovery, dict) or discovery.get('parameter') not in keys or
                    not isinstance(discovery.get('command'), list) or not discovery['command'] or
                    any(not isinstance(v, str) or not v or '\0' in v for v in discovery['command'])):
                raise ValueError(f'{identity}: invalid device_discovery command/parameter')
            target = next(d for d in tool['parameters'] if d['key'] == discovery['parameter'])
            if target['type'] != 'string' or not target.get('device_selector'):
                raise ValueError(f'{identity}: device discovery needs a string device_selector parameter')
        env = tool.get('environment', {})
        if not isinstance(env, dict) or any(not IDENTIFIER.fullmatch(k) or not isinstance(v, str) or '\0' in v for k, v in env.items()):
            raise ValueError(f'{identity}: invalid environment')


class ConfigStore:
    def __init__(self, path=None, repository=REPOSITORY):
        self.path = Path(path or config_path()).expanduser().resolve()
        self.repository = Path(repository).resolve()
        self.data = {}
        self.digest = None

    def load(self):
        if self.path.exists():
            raw = self.path.read_bytes()
            data = json.loads(raw)
            validate_config(data)
            self.digest = hashlib.sha256(raw).hexdigest()
            self.data = data
        else:
            self.data = default_config(self.repository)
            self.save(self.data)
        return self.data

    def save(self, data):
        validate_config(data)
        current = hashlib.sha256(self.path.read_bytes()).hexdigest() if self.path.exists() else None
        if current != self.digest:
            raise ValueError('Configuration changed externally. Reload first / 配置已被外部修改，请先重新读取')
        atomic_json(self.path, data)
        self.digest = hashlib.sha256(self.path.read_bytes()).hexdigest()
        self.data = copy.deepcopy(data)


def launch_spec(tool, repository=REPOSITORY):
    substitutions = {'repo': str(repository)}
    for definition in tool.get('parameters', []):
        value = validate_value(definition, parameter_value(tool, definition))
        substitutions[definition['key']] = ('1' if value else '0') if type(value) is bool else str(value)

    def expand(text):
        # Restrict placeholders to simple names, never attribute/index lookups.
        import string
        for _, field, fmt, conversion in string.Formatter().parse(text):
            if field is not None and (not IDENTIFIER.fullmatch(field) or fmt or conversion):
                raise ValueError(f'Invalid command placeholder: {text}')
        try:
            return text.format_map(substitutions)
        except KeyError as exc:
            raise ValueError(f'Unknown command placeholder: {exc}') from exc

    env = {k: expand(v) for k, v in tool.get('environment', {}).items()}
    for definition in tool.get('parameters', []):
        if definition.get('env'):
            env[definition['env']] = substitutions[definition['key']]
    paths = [expand(p) for p in tool.get('python_paths', [])]
    if paths:
        env['PYTHONPATH'] = os.pathsep.join(paths + ([os.environ['PYTHONPATH']] if os.environ.get('PYTHONPATH') else []))
    env.update(PYTHONUNBUFFERED='1', PYTHONFAULTHANDLER='1')
    return {'name': tool['name'], 'command': [expand(p) for p in tool['command']],
            'cwd': str(Path(expand(tool['cwd'])).expanduser().resolve()), 'environment': env,
            'exclusive_module': tool.get('exclusive_module')}


def check_spec(spec):
    if not Path(spec['cwd']).is_dir():
        raise ValueError(f"Working directory missing / 工作目录不存在: {spec['cwd']}")
    executable = spec['command'][0]
    if '/' in executable:
        path = Path(executable)
        if not path.is_absolute():
            path = Path(spec['cwd'])/path
        valid = path.is_file() and os.access(path, os.X_OK)
    else:
        valid = bool(shutil.which(executable))
    if not valid:
        raise ValueError(f'Executable missing / 解释器或程序不存在: {executable}')
