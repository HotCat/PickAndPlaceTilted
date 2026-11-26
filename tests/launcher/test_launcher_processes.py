"""Real child/supervisor lifetimes, streamed logs and independent instance locks."""
import sys
import time

import pytest

from launcher.processes import AlreadyRunning, ProcessManager


def wait_until(predicate, timeout=8):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.03)
    raise AssertionError('Timed out waiting for supervisor state/logs')


def spec(tmp_path):
    return {'name':'Test tool', 'command':[sys.executable,'-u','-c',
            'import os,signal,time,sys; signal.signal(signal.SIGTERM,lambda *_:sys.exit(0)); '
            'print("工具 ready",os.environ["EXAMPLE_PARAMETER"],flush=True); time.sleep(120)'],
            'cwd':str(tmp_path),'environment':{'EXAMPLE_PARAMETER':'test value'}}


def test_reopen_manager_duplicate_lock_live_logs_and_graceful_stop(tmp_path):
    first = ProcessManager(tmp_path/'state')
    first.launch('tool', spec(tmp_path))
    second = ProcessManager(first.directory)
    try:
        wait_until(lambda:'工具 ready test value' in second.log_tail('tool'))
        assert second.status('tool')['running']
        with pytest.raises(AlreadyRunning):
            second.launch('tool',spec(tmp_path))
        second.stop('tool')
        wait_until(lambda:not second.status('tool')['running'])
        assert second.status('tool')['exit_code'] == 0
        assert 'Graceful close requested' in second.log_tail('tool')
        second.launch('tool',spec(tmp_path))
        wait_until(lambda:second.log_tail('tool').count('工具 ready') == 2)
    finally:
        second.stop('tool')
        wait_until(lambda:not second.status('tool')['running'])


def test_invalid_executable_is_actionable_without_leaving_running_lock(tmp_path):
    manager = ProcessManager(tmp_path/'state')
    bad = spec(tmp_path)
    bad['command'][0] = '/no/such/python'
    with pytest.raises(ValueError, match='Executable missing'):
        manager.launch('bad',bad)
    assert not manager.status('bad')['running']


def test_exit_code_and_early_stderr_are_retained(tmp_path):
    manager = ProcessManager(tmp_path/'state')
    command = spec(tmp_path)
    command['command'] = [sys.executable,'-c','import sys; print("failure detail",file=sys.stderr); sys.exit(7)']
    manager.launch('failure',command)
    wait_until(lambda:not manager.status('failure')['running'])
    assert manager.status('failure')['exit_code'] == 7
    assert manager.status('failure')['state'] == 'failed'
    assert 'failure detail' in manager.log_tail('failure')


def test_unmanaged_module_is_detected_and_not_signalled(tmp_path):
    import os
    from launcher.processes import external_instances
    assert os.getpid() in external_instances('pytest')
    manager=ProcessManager(tmp_path/'state')
    command=spec(tmp_path)
    command['exclusive_module']='pytest'
    with pytest.raises(AlreadyRunning,match='outside this launcher'):
        manager.launch('external',command)
    assert not manager.status('external')['running']
