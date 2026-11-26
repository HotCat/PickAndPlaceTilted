# Camera preview / 相机预览

The standalone preview reuses the PCB registration tool's camera window, without
loading CAD, rectification, registration or robot control. Choose either SDK at
launch. Device enumeration, acquisition and parameter writes run on one worker
thread; the GUI never owns native camera buffers.

Use **Intrinsic calibration / 相机内参标定…** in the preview to collect chessboard
views, fit K/distortion, inspect pixel errors and export profiles. See the
[intrinsic calibration guide / 内参标定说明](camera-intrinsics.md) for the verified
11 × 8 inner-corner, 3 mm board and the collection workflow.

## Launch on this workstation / 本机启动

```bash
cd /home/hotcat/Downloads/fr3robot/PickAndPlaceTilted
./preview-camera.sh
```

The default backend is **MindVision**, using the **installed `libMVSDK.so`**, the
same library loaded by the working GTK demo (`/usr/lib/libMVSDK.so` here).
The library under the unpacked `linuxSDK_V2.1.0.6/lib/x64/` has a different hash
and finds zero devices on this workstation, while the installed version detects
the `MV-GE502M-T`. The launcher therefore no longer forces the unpacked SDK root.
No system files are modified. The scan status shows the library being loaded.

默认使用 GTK 演示程序所用的已安装 SDK。本机压缩包内的库与已安装库不同，前者无法
检测到这台相机，因此启动脚本已取消强制使用该目录。

Select the device → **Open camera** → **Start live view**. Use resolution, exposure,
auto exposure, gain, gamma, contrast and mirror controls. Click
**Apply parameters** to change them. Opening does not overwrite exposure or gain.
MindVision gain uses native integer SDK units; Hikvision gain uses dB.

For **manual exposure**, click **Manual exposure / 手动曝光** (or uncheck Auto
exposure), enter Exposure in microseconds, then click **Apply parameters**.
Gamma and contrast use the same native scales as GTK_demo (100 is neutral on
this camera). Applying them selects the SDK's parameter-generated LUT so their
changes affect the processed image.

The **Resolution / 分辨率** list comes from `CameraGetCapability`, just like
GTK_demo. It selects a real acquisition preset with its complete ROI/binning
configuration, not just display resizing. Acquisition pauses while settings are
applied and then resumes if it was running. This GE502M exposes:

| Preset | Mode |
| --- | --- |
| 2592 × 1944 | Full 5 MP |
| 2048 × 1536 | ROI |
| 1920 × 1080 | ROI |
| 1280 × 1024 | ROI |
| 640 × 480 | ROI |
| 1296 × 972 | BIN2X2 |

The preview image is scaled to fit the window; native acquisition uses the
selected preset. ROI can crop the field of view. Recheck the fiducial/crosshair
alignment when changing presets; a resolution change is not a camera calibration.

选择相机 → **Open camera（打开）** → **Start live view（实时预览）**。
可选择分辨率，调整曝光、自动曝光、增益、伽马、对比度和镜像，然后点击
**Apply parameters** 应用。手动曝光请点击 **Manual exposure / 手动曝光**，输入
微秒值后应用；此操作关闭自动曝光。分辨率列表与 GTK_demo 一样来自相机 SDK，实际
修改采集模式，而非仅缩放显示。ROI 可能裁剪视野，切换后请重新检查十字线对准。
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
# Force the same exact library as this workstation's GTK demo:
./preview-camera.sh --sdk-library /usr/lib/libMVSDK.so
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

Verified MindVision acquisition from `MV-GE502M-T (051020320321)`, a monochrome
2592 × 1944 camera, using the installed SDK. Close GTK_demo before opening it in
our preview: camera acquisition requires exclusive access. The unrelated USB
Hikvision `MV-CB013-20UM-C` remains available through the Hikvision backend.

## Tests

```bash
.venv-camera/bin/python -m pip install pytest
QT_QPA_PLATFORM=offscreen .venv-camera/bin/python -m pytest \
  tests/test_mindvision_camera.py tests/test_hikvision_camera.py tests/test_camera_window.py
```

Mock SDK tests cover native-buffer release on ISP failure, independent frame
ownership, mono/color output, transport errors versus ordinary timeouts, snapshot
freshness, parameter errors and cleanup after disconnect/partial initialization.
