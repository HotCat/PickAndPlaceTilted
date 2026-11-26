# Camera preview / 相机预览

The standalone preview reuses the PCB registration tool's camera window, without
loading CAD, rectification, registration or robot control. Choose either SDK at
launch. Device enumeration, acquisition and parameter writes run on one worker
thread; the GUI never owns native camera buffers.

## Launch on this workstation / 本机启动

```bash
cd /home/hotcat/Downloads/fr3robot/PickAndPlaceTilted
./preview-camera.sh
```

The default backend is **MindVision**. The launcher uses the supplied SDK at
`~/Downloads/new3dvision/library/hopencv/mindvision2/usv-multiSensor-driver-master/linuxSDK_V2.1.0.6/`.
It loads `lib/x64/libMVSDK.so` directly on this x86-64 workstation: no copying to
`/lib`, administrator launch or system SDK installation is needed.

Select the device → **Open camera** → **Start live view**. Use exposure, auto
exposure, gain, gamma and mirror controls as supported by the device. Click
**Apply parameters** to change them. Opening does not overwrite exposure or gain.
MindVision gain uses native integer SDK units; Hikvision gain uses dB.

选择相机 → **Open camera（打开）** → **Start live view（实时预览）**。
可调整曝光、自动曝光、增益、伽马和镜像，然后点击 **Apply parameters** 应用。
默认显示中心十字线，便于粗略对准标记；十字线只叠加在显示画面上。

For the existing Hikvision camera / 使用现有海康相机：

```bash
./preview-camera.sh --backend hikvision
```

Hikvision uses the installed MVS SDK at `/opt/MVS`; override with
`HIKVISION_MVS_ROOT` if needed. The full PCB registration application remains
available via `python tools/select_pcb_holes.py`; its existing camera button still
opens Hikvision capture for registration.

## Recreate environment / 重建环境

The dedicated `.venv-camera` is independent of the robot's `.venv`:

```bash
/usr/bin/python3 -m venv .venv-camera
.venv-camera/bin/python -m pip install -r requirements-preview.txt
```

The preview uses PySide6 6.8.3 and headless OpenCV (OpenCV is only used for image
processing). This avoids competing Qt plugin paths and a Qt 6.12 shutdown heap
error reproduced with the existing camera-window tests on this machine.

To use a different SDK / 指定其他 SDK：

```bash
./preview-camera.sh --sdk-root /path/to/linuxSDK_V2.1.0.6
# Equivalent direct entry point:
.venv-camera/bin/python -m pcbvision.preview --backend mindvision --sdk-root /path/to/linuxSDK
```

`MINDVISION_SDK_ROOT` selects the architecture subdirectory in `lib/`;
`MINDVISION_SDK_LIBRARY` overrides the exact `.so` path. Libraries are not bundled
in Git. The older `app.py` MindVision rectification workbench also honors these
environment variables through the shared vendor wrapper.

## Aiming and detection / 对准与设备检测

The crosshair marks the **image center**, not a calibrated TCP projection. A
TCP-mounted camera has an optical offset, lens distortion and an aiming error
that changes with working distance. Preview supports rough fiducial aiming;
accurate robot coordinates require camera/hand–eye calibration or a separately
validated fixed working-distance offset. This tool does not change robot planes
or move the robot.

十字线表示图像中心，并不等同于 TCP 投影。相机与 TCP 的偏移、镜头畸变及工作距离
都会影响瞄准精度。本工具适合粗定位；精确标定还需手眼标定或经过验证的固定工作距离
偏移补偿。预览程序不会连接机器人，也不会修改工件平面。

If no cameras are found, click **Find cameras** after checking power/cable and
closing vendor camera applications. For GigE cameras check the host interface
and camera subnet; for USB cameras check USB detection and device permissions.
The SDK's `88-mvusb.rules` may be needed on systems without camera USB permissions;
it is not installed automatically. SDK loading and finding a physical device are
separate checks.

At implementation verification the supplied MindVision SDK loaded successfully
but enumerated zero devices. Hikvision MVS enumerated `GE502M` and the USB
`MV-CB013-20UM-C`; the latter delivered a 1280 × 1024 BGR live frame. A physical
MindVision acquisition still needs verification when its device is discoverable.

## Tests

```bash
.venv-camera/bin/python -m pip install pytest
QT_QPA_PLATFORM=offscreen .venv-camera/bin/python -m pytest \
  tests/test_mindvision_camera.py tests/test_hikvision_camera.py tests/test_camera_window.py
```

Mock SDK tests cover native-buffer release on ISP failure, independent frame
ownership, mono/color output, transport errors versus ordinary timeouts, snapshot
freshness, parameter errors and cleanup after disconnect/partial initialization.
