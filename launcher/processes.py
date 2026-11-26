"""Detached Linux supervisors: durable rotating logs and one live instance per key.

The lock is inherited by the app too, so a supervisor failure cannot accidentally
permit a second robot studio. Stop is a per-run request consumed by the original
supervisor, never a kill of a PID read from an old status file. There is no forced
kill or automatic restart: built-in Qt apps handle SIGTERM through closeEvent.
"""
from __future__ import annotations

import argparse
import codecs
from datetime import datetime
import fcntl
import hashlib
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import uuid

from .config import REPOSITORY, atomic_json, check_spec, state_path


def timestamp():
    return datetime.now().astimezone().isoformat(timespec='seconds')


class AlreadyRunning(ValueError):
    pass


def external_instances(module):
    """Detect older same-user terminal launches; do not signal or adopt them."""
    if not module:
        return []
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            argv = (entry/'cmdline').read_bytes().decode(errors='replace').split('\0')
            if any(argv[i] == '-m' and argv[i+1] == module for i in range(len(argv)-1)):
                found.append(int(entry.name))
        except (OSError, ProcessLookupError):
            continue
    return found


class ProcessManager:
    def __init__(self, directory=None):
        self.directory = Path(directory or state_path()).expanduser().resolve()
        self.directory.mkdir(mode=0o700, parents=True, exist_ok=True)

    def paths(self, key):
        root = self.directory/hashlib.sha256(key.encode()).hexdigest()
        return {name: root.with_suffix(suffix) for name, suffix in
                (('lock', '.lock'), ('record', '.json'), ('log', '.log'), ('control', '.control'))}

    def status(self, key):
        paths = self.paths(key)
        with paths['lock'].open('a+b') as stream:
            try:
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
                running = False
            except BlockingIOError:
                running = True
            try:
                record = json.loads(paths['record'].read_text())
            except (OSError, ValueError):
                record = {}
        record['running'] = running
        record['log'] = str(paths['log'])
        if running:
            if record.get('state') not in ('starting', 'running', 'stopping'):
                record['state'] = 'running'
        elif record.get('state') in ('starting', 'running', 'stopping'):
            record.update(state='exited', message='Process ended / 进程已结束')
        else:
            record.setdefault('state', 'ready')
        return record

    def launch(self, key, spec):
        check_spec(spec)
        paths = self.paths(key)
        descriptor = os.open(paths['lock'], os.O_CREAT | os.O_RDWR, 0o600)
        request = None
        try:
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise AlreadyRunning('Already running / 程序已运行') from exc
            external = external_instances(spec.get('exclusive_module'))
            if external:
                raise AlreadyRunning(f'An existing window is running outside this launcher (PID {external}). Close it normally first. / 请先正常关闭原有工具窗口。')
            run_id = uuid.uuid4().hex
            initial = dict(state='starting', run_id=run_id, name=spec['name'],
                           started_at=timestamp(), command=spec['command'], cwd=spec['cwd'])
            atomic_json(paths['record'], initial)
            request = self.directory/f'request-{run_id}.json'
            atomic_json(request, dict(spec=spec, initial=initial,
                                     paths={k: str(v) for k, v in paths.items()}))
            supervisor = subprocess.Popen(
                [sys.executable, '-m', 'launcher.processes', '--request', str(request),
                 '--lock-fd', str(descriptor)], cwd=REPOSITORY,
                env=dict(os.environ, PYTHONPATH=str(REPOSITORY)),
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True, close_fds=True, pass_fds=(descriptor,))
            threading.Thread(target=supervisor.wait, daemon=True, name='tool-supervisor-reaper').start()
        except Exception as exc:
            if request:
                request.unlink(missing_ok=True)
                atomic_json(paths['record'], dict(initial, state='failed', message=str(exc)))
            raise
        finally:
            # Closing only our fd preserves the inherited open-file lock.
            os.close(descriptor)

    def stop(self, key):
        record = self.status(key)
        if record['running'] and record.get('run_id'):
            atomic_json(self.paths(key)['control'], {'run_id': record['run_id'], 'action': 'stop'})

    def log_tail(self, key, limit=128*1024):
        try:
            with self.paths(key)['log'].open('rb') as stream:
                stream.seek(0, os.SEEK_END)
                stream.seek(max(0, stream.tell()-limit))
                return stream.read().decode('utf-8', errors='replace')
        except FileNotFoundError:
            return ''


def supervise(request, descriptor):
    payload = json.loads(Path(request).read_text())
    Path(request).unlink(missing_ok=True)
    spec, record, paths = payload['spec'], payload['initial'], payload['paths']
    handler = RotatingFileHandler(paths['log'], maxBytes=2*1024*1024, backupCount=3, encoding='utf-8')
    handler.terminator = ''
    logger = logging.getLogger('workbench-supervisor')
    logger.setLevel(logging.INFO)
    logger.addHandler(handler)
    stop_requested = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop_requested.set())
    signal.signal(signal.SIGINT, lambda *_: stop_requested.set())
    logger.info('\n=== %s | %s ===\n', timestamp(), spec['name'])
    logger.info('Source: %s\nCommand: %s\n', spec['cwd'], json.dumps(spec['command'], ensure_ascii=False))
    child = None
    try:
        environment = dict(os.environ, **spec.get('environment', {}))
        child = subprocess.Popen(spec['command'], cwd=spec['cwd'], env=environment,
                                 stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, bufsize=0, close_fds=True, pass_fds=(descriptor,))
        record.update(state='running', pid=child.pid, supervisor_pid=os.getpid())
        atomic_json(paths['record'], record)

        def collect():
            decoder = codecs.getincrementaldecoder('utf-8')(errors='replace')
            while chunk := child.stdout.read(8192):
                logger.info('%s', decoder.decode(chunk))
            logger.info('%s', decoder.decode(b'', final=True))

        reader = threading.Thread(target=collect, daemon=True, name='tool-log-collector')
        reader.start()
        sent = False
        while child.poll() is None:
            try:
                control = json.loads(Path(paths['control']).read_text())
                if control.get('run_id') == record['run_id'] and control.get('action') == 'stop':
                    stop_requested.set()
            except (OSError, ValueError):
                pass
            if stop_requested.is_set() and not sent:
                try:
                    child.send_signal(signal.SIGTERM)
                except ProcessLookupError:
                    pass
                sent = True
                record.update(state='stopping', message='Waiting for normal application shutdown / 等待程序正常退出')
                atomic_json(paths['record'], record)
                logger.info('\n[%s] Graceful close requested; waiting (no forced kill).\n', timestamp())
            try:
                child.wait(timeout=.15)
            except subprocess.TimeoutExpired:
                pass
        reader.join(timeout=3)
        record.update(state='exited' if child.returncode == 0 else 'failed',
                      exit_code=child.returncode, ended_at=timestamp())
        logger.info('\n[%s] Process exited: %s\n', timestamp(), child.returncode)
        return child.returncode
    except Exception as exc:
        record.update(state='failed', message=str(exc), ended_at=timestamp())
        logger.exception('Supervisor failure: %s\n', exc)
        return 1
    finally:
        atomic_json(paths['record'], record)
        handler.close()
        os.close(descriptor)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--request', required=True, type=Path)
    parser.add_argument('--lock-fd', required=True, type=int)
    args = parser.parse_args()
    raise SystemExit(supervise(args.request, args.lock_fd))
