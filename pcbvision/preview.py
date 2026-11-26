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
    args = parser.parse_args(argv)
    if args.sdk_root:
        os.environ["MINDVISION_SDK_ROOT"] = str(args.sdk_root.expanduser().resolve())
    if args.sdk_library:
        os.environ["MINDVISION_SDK_LIBRARY"] = str(args.sdk_library.expanduser().resolve())

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
    window = CameraWindow(camera_factory=factory, camera_name=name, preview_only=True)
    window.setAttribute(Qt.WA_DeleteOnClose)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
