"""Cooperative shutdown for managed Qt tools, with their normal closeEvent cleanup."""
import signal

from PySide6.QtCore import QTimer


def install_close_handler(app):
    # Python signal handlers only set a flag. Qt closes windows on its own event
    # loop, so SDK workers and the FR3 stop channel run their existing cleanup.
    requested = []
    previous = {}
    for signum in (signal.SIGTERM, signal.SIGINT):
        previous[signum] = signal.getsignal(signum)
        signal.signal(signum, lambda *_: requested.append(True))
    timer = QTimer(app)
    timer.setInterval(100)

    def poll():
        if requested:
            requested.clear()
            print('Launcher requested normal window close / 启动器请求正常退出', flush=True)
            app.closeAllWindows()

    timer.timeout.connect(poll)
    timer.start()
    # Hold the timer/closure for the full application lifetime.
    app._workbench_close_timer = timer
    return previous
