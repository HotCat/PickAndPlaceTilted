"""Desktop entry point for the pick/place tool launcher."""
import argparse
from pathlib import Path
import sys

from .config import ConfigStore, REPOSITORY, config_path


def main(argv=None):
    parser = argparse.ArgumentParser(description='Pick & Place Workbench / 取放工作台启动器')
    parser.add_argument('--config', type=Path, default=config_path())
    parser.add_argument('--repo', type=Path, default=REPOSITORY)
    parser.add_argument('--check-config', action='store_true')
    parser.add_argument('--smoke-test', action='store_true', help='Show launcher only, then exit without starting tools')
    parser.add_argument('--screenshot', type=Path)
    args = parser.parse_args(argv)
    store = ConfigStore(args.config, args.repo)
    if args.check_config:
        store.load()
        print(f'OK: {len(store.data["tools"])} tools; {store.path}')
        return 0
    from PySide6.QtCore import QLockFile, QTimer
    from PySide6.QtWidgets import QApplication, QMessageBox
    from .ui import LauncherWindow
    app = QApplication(sys.argv[:1])
    app.setStyle('Fusion')
    lock = QLockFile(str(store.path) + '.ui.lock')
    store.path.parent.mkdir(parents=True, exist_ok=True)
    lock.setStaleLockTime(0)
    if not lock.tryLock(0):
        QMessageBox.information(None, 'Launcher already open / 启动器已打开', 'Use the existing launcher window / 请使用现有启动器窗口')
        return 0
    try:
        store.load()
    except (ValueError, OSError, TypeError, KeyError) as exc:
        QMessageBox.critical(None, 'Configuration error / 配置错误', f'{store.path}\n{exc}')
        return 1
    window = LauncherWindow(store)
    window.show()
    if args.smoke_test:
        def finish():
            if args.screenshot:
                window.grab().save(str(args.screenshot))
            window.close()
        QTimer.singleShot(600, finish)
    result = app.exec()
    lock.unlock()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
