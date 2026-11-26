"""Catalogue validation and argv/environment mapping without hardware."""
import copy
import json
from pathlib import Path

import pytest

from launcher.config import ConfigStore, default_config, launch_spec, validate_config


def test_defaults_reuse_legacy_environment_and_calibration_but_import_repository(tmp_path):
    repo = tmp_path/'repository'
    repo.mkdir()
    (tmp_path/'.venv/bin').mkdir(parents=True)
    (tmp_path/'.venv/bin/python').touch()
    data = default_config(repo)
    validate_config(data)
    studio = data['tools'][1]
    spec = launch_spec(studio, repo)
    assert spec['command'][0] == str(tmp_path/'.venv/bin/python')
    assert str(tmp_path/'calibration-session.json') in spec['command']
    assert spec['environment']['PYTHONPATH'].split(':')[0] == str(repo/'fr3-cartesian')
    assert '--robot-ip' in spec['command'] and '--backend' in spec['command']
    assert '--auto-connect' not in spec['command']


def test_values_are_literal_argv_and_not_shell_interpreted(tmp_path):
    tool = default_config()['tools'][0]
    weird = 'camera with spaces; $(touch /tmp/not-executed)'
    tool['values']['camera_serial'] = weird
    spec = launch_spec(tool, tmp_path)
    assert weird in spec['command']
    assert spec['environment']['PICKPLACE_CAMERA_AUTO_PREVIEW'] == '1'
    tool['values']['auto_preview'] = False
    assert launch_spec(tool, tmp_path)['environment']['PICKPLACE_CAMERA_AUTO_PREVIEW'] == '0'


@pytest.mark.parametrize('field,value', [('robot_ip','999.1.1.1'),('backend','invalid'),('language',123)])
def test_invalid_robot_settings_are_rejected(field, value):
    data = default_config()
    data['tools'][1]['values'][field] = value
    with pytest.raises(ValueError):
        validate_config(data)


def test_json_adds_a_tool_without_ui_or_process_code_changes(tmp_path):
    data = default_config(tmp_path)
    data['tools'].append({'id':'custom-inspector','name':'Inspector','cwd':'{repo}',
                          'command':['/usr/bin/python3','-u','inspect.py','--port','{port}'],
                          'parameters':[{'key':'port','label':'Port','type':'integer',
                                         'min':1,'max':65535,'default':8000,'env':'INSPECT_PORT'}]})
    validate_config(data)
    spec = launch_spec(data['tools'][-1], tmp_path)
    assert spec['command'][-1] == '8000'
    assert spec['environment']['INSPECT_PORT'] == '8000'


def test_external_edit_is_not_overwritten_and_reload_preserves_extension(tmp_path):
    store = ConfigStore(tmp_path/'config.json', tmp_path)
    store.load()
    changed = copy.deepcopy(store.data)
    changed['tools'][0]['description'] = 'External change'
    store.path.write_text(json.dumps(changed))
    with pytest.raises(ValueError, match='externally'):
        store.save(store.data)
    store.load()
    assert store.data['tools'][0]['description'] == 'External change'
    store.data['tools'][0]['values']['camera_serial'] = '123'
    store.save(store.data)
    assert json.loads(store.path.read_text())['tools'][0]['values']['camera_serial'] == '123'


def test_command_template_cannot_access_python_attributes():
    tool = default_config()['tools'][0]
    tool['command'] = ['{repo.__class__}']
    with pytest.raises(ValueError, match='placeholder'):
        launch_spec(tool)
