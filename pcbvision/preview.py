"""Preview-only entry point, independent of CAD dependencies and robot software.

python -m pcbvision.preview --backend mindvision --sdk-root /path/to/linuxSDK
python -m pcbvision.preview --backend hikvision
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description="Industrial-camera aiming preview (no robot connection)")
    parser.add_argument("--backend", choices=("mindvision", "hikvision"), default="mindvision")
    parser.add_argument("--sdk-root", type=Path, help="Unpacked MindVision SDK directory containing lib/x64/libMVSDK.so")
    parser.add_argument("--sdk-library", type=Path, help="Exact MindVision shared library (default: installed libMVSDK.so, same as GTK_demo)")
    parser.add_argument("--device", default="", help="Select the exact camera serial number")
    parser.add_argument("--auto-preview", action="store_true", default=os.environ.get('PICKPLACE_CAMERA_AUTO_PREVIEW') == '1',
                        help="Open selected camera and start preview (does not connect any robot)")
    parser.add_argument("--log-stdout", action="store_true", help="Publish status/errors to launcher logs")
    parser.add_argument("--list-devices-json", action="store_true", help="Enumerate cameras without opening them (launcher discovery)")
    args = parser.parse_args(argv)
    if args.sdk_root:
        os.environ["MINDVISION_SDK_ROOT"] = str(args.sdk_root.expanduser().resolve())
    if args.sdk_library:
        os.environ["MINDVISION_SDK_LIBRARY"] = str(args.sdk_library.expanduser().resolve())
    if args.list_devices_json:
        import json
        if args.backend == 'mindvision':
            from .mindvision import MindVisionCamera
            camera = MindVisionCamera()
        else:
            from .hikvision import HikvisionCamera
            camera = HikvisionCamera()
        try:
            devices = [{k: d[k] for k in ('name', 'sn')} for d in camera.enumerate_devices()]
            print('CAMERA_DEVICES_JSON:' + json.dumps(devices, ensure_ascii=False), flush=True)
        finally:
            camera.close()
        return 0

    # Import OpenCV before Qt, then remove opencv-python's conflicting Qt plugin
    # hint. This window uses PySide6's Qt6 plugins; OpenCV's Qt5 plugins can abort
    # startup on Linux even though no cv2.imshow() is ever used here.
    import cv2
    for key in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR"):
        if "cv2" in os.environ.get(key, ""):
            os.environ.pop(key, None)

    from PySide6.QtCore import Qt
    from PySide6.QtGui import QColor, QPalette
    from PySide6.QtWidgets import QApplication
    from .camera_window import CameraWindow
    if args.backend == "mindvision":
        from .mindvision import MindVisionCamera
        factory, name = MindVisionCamera, "MindVision"
    else:
        from .hikvision import HikvisionCamera
        factory, name = HikvisionCamera, "Hikvision"

    app = QApplication.instance() or QApplication(sys.argv[:1])
    # Explicit palette avoids white controls with light text on dark desktops.
    app.setStyle("Fusion")
    palette = QPalette()
    for role, color in ((QPalette.Window, "#252a32"), (QPalette.WindowText, "#eeeeee"),
                        (QPalette.Base, "#171b20"), (QPalette.AlternateBase, "#303641"),
                        (QPalette.Text, "#eeeeee"), (QPalette.Button, "#39414d"),
                        (QPalette.ButtonText, "#eeeeee"), (QPalette.Highlight, "#3276b8"),
                        (QPalette.HighlightedText, "#ffffff"),
                        (QPalette.ToolTipBase, "#252a32"), (QPalette.ToolTipText, "#eeeeee")):
        palette.setColor(role, QColor(color))
    for role in (QPalette.Text, QPalette.WindowText, QPalette.ButtonText):
        palette.setColor(QPalette.Disabled, role, QColor("#8c95a3"))
    app.setPalette(palette)
    window = CameraWindow(camera_factory=factory, camera_name=name, preview_only=True,
                          device_serial=args.device, auto_preview=args.auto_preview,
                          log_stdout=args.log_stdout)
    if os.environ.get('WORKBENCH_MANAGED') == '1':
        from launcher.runtime import install_close_handler
        install_close_handler(app)
    window.setAttribute(Qt.WA_DeleteOnClose)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
