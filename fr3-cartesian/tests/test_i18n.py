"""Verify translated operator messages, including the process protocol."""
import ast
import json
from pathlib import Path
import string
import subprocess
import sys

from fr3_cartesian.i18n import ZH_CN, translate
from fr3_cartesian.messages import Message, OperatorValueError, error_message


def fields(text):
    return sorted((field, spec, conversion) for _, field, spec, conversion in string.Formatter().parse(text)
                  if field is not None)


def test_catalog_preserves_all_named_values_and_precision():
    for source, translated in ZH_CN.items():
        assert translated.strip(), source
        assert fields(source) == fields(translated), source


def test_every_static_operator_message_has_a_translation():
    root = Path(__file__).resolve().parents[1]/'fr3_cartesian'
    functions = {'OperatorError', 'OperatorValueError', 'MoveRejected', 'Message'}
    methods = {'label', 'paragraph', 'pushbutton', 'button', 'checkbox', 'group', 'add_tab', 'log', 'tr'}
    missing = set()
    for file in root.glob('*.py'):
        if file.name in ('i18n.py', 'messages.py'):
            continue
        for node in ast.walk(ast.parse(file.read_text())):
            if not isinstance(node, ast.Call) or not node.args:
                continue
            name = node.func.id if isinstance(node.func, ast.Name) else None
            method = node.func.attr if isinstance(node.func, ast.Attribute) else None
            if name not in functions and method not in methods:
                continue
            first = node.args[1] if method == 'add_tab' else node.args[0]
            if isinstance(first, ast.Constant) and isinstance(first.value, str):
                if first.value not in ZH_CN:
                    missing.add(first.value)
    assert not missing


def test_numeric_errors_roundtrip_and_translate_in_either_language():
    message = error_message(OperatorValueError(
        'Contact residual {maximum:.3f} mm exceeds {tolerance:.3f} mm; recapture the points',
        maximum=1.23456, tolerance=.5))
    nested = Message('ERROR: {reason}', {'reason': message})
    restored = Message.from_payload(json.loads(json.dumps(nested.payload())))
    assert translate('en', restored) == 'ERROR: Contact residual 1.235 mm exceeds 0.500 mm; recapture the points'
    assert translate('zh_CN', restored) == '错误：接触残差 1.235 mm 超过 0.500 mm，请重新采集点位'


def test_worker_returns_translatable_errors_with_original_error_code():
    request = json.dumps({'id': 1, 'method': 'start_jog', 'args': {'ref':0, 'axis':1, 'direction':1, 'distance':20, 'speed':3}})
    result = subprocess.run([sys.executable, '-m', 'fr3_cartesian.worker'], input=request+'\n',
                            capture_output=True, text=True, timeout=5, check=True)
    error = Message.from_payload(json.loads(result.stdout)['error'])
    assert translate('en', error) == 'Jog travel is out of range or speed exceeds 10%'
    assert translate('zh_CN', error) == '点动行程超出范围或速度超过 10%'
    vendor = Message('{name}: FAIRINO error {code}', {'name':'MoveL', 'code':112})
    assert translate('zh_CN', vendor) == 'MoveL：FAIRINO 错误码 112'
