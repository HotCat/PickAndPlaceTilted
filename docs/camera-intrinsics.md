# Camera intrinsic calibration / 相机内参标定

Open **Pick & Place Workbench → Camera preview → Start**, then click
**Intrinsic calibration / 相机内参标定…** underneath the camera open/live controls.
If preview is already running an older version, use **Restart** in the launcher.
The robot is not connected or moved by this feature.

The live camera was checked: **MV-GE502M-T, serial 051020320321**, monochrome,
2592 × 1944. OpenCV detected all **88 inner corners** on the board marked
`GP050 12*9-3`: 12 × 9 squares correspond to **11 × 8 inner corners**; square
side is **3 mm**. These are the new window's defaults. The older `app.py`
rectification tool's independent 5 mm default is not used here.

## Collecting a dataset

1. Choose the camera's resolution preset **before** collecting. Use the intended
   deployment mode; 2592 × 1944 full resolution is the initial setup. Fix focus,
   aperture and lens mounting. Enter an identifying lens/focus description and
   check **Focus/aperture fixed; board flat**. If focus changes, start a new session.
2. Keep the full chessboard sharp and visible with some border around it. Avoid
   glare, motion blur and bending the target. Exposure/gain can be adjusted to
   keep images usable; the target must be stationary during each capture.
3. Click **Capture view / 采集当前视图**. A fresh full-resolution frame is acquired
   on the camera worker, then detected/refined on another worker. The selected
   sample displays all 88 colored corners. GUI preview resizing/crosshairs never
   enter the calibration measurements. Raw images and refined corners are saved
   immediately; closing the program does not lose accepted samples.
   Enable **Live framing / 实时取景** while arranging the next view (start live
   view in the camera window first). Capture switches back to the saved native
   image/corner overlay. Live framing is display-only and does not add samples.
4. Collect **20–30 different views**, with the board near the image center, edges
   and corners, at several distances, and tilted roughly 15–35° about **both**
   board axes where detection/focus permit. Include some larger board images to
   constrain distortion. Keep focus fixed. In-plane rotation alone is inadequate.
   The current board occupies only about 5% of the image, so include closer/larger
   views and more image positions rather than repeating this one view.
5. With the robot stationary, move the board by hand if practical. This window
   sends no robot commands. Moving a wrist-mounted camera through varied poses
   using separate teaching controls is also possible, but automatic robot motion
   collection and robot/camera synchronization are outside this feature.
6. **Fit intrinsics / 计算内参** becomes available after at least **10 enabled**
   views. Ten is a minimum, not an accuracy guarantee. Near-duplicate observations
   are rejected; geometrically degenerate/front-parallel datasets are rejected
   even if a numerical fit could produce a deceptively small error.

The first live view used for verification is retained at:

```text
~/.local/share/pickplace-workbench/camera-calibration/ge502m-11x8-3mm-initial/
```

The window restores the last saved session automatically. Confirm the lens/focus
setup is unchanged before adding views. This initial view is **not a completed
calibration**. New sessions receive a separate timestamped directory; older data
is preserved. Use **Resume session / 恢复采集** to choose another session folder.

## Inspecting and exporting

The fit uses OpenCV's pinhole model with five Brown distortion coefficients:

```text
K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
D = [k1, k2, p1, p2, k3]
```

The window reports global RMS and each view's RMS in **native image pixels**,
focal lengths/principal point, center-position coverage, corner hull coverage,
and fitted board-normal diversity. The saved result also contains intrinsic
parameter standard deviations and per-view board-to-camera extrinsics.

Inspect high-error samples in the list. Uncheck an unusable sample to exclude
it and refit. Images remain on disk; there is no silent removal of outliers.
Adding/excluding views invalidates the previous result until a new fit completes.
Warnings identify limited coverage, insufficient second-axis tilt, fewer than
20 views, large residuals, implausible parameters and large focal uncertainty.
A low training residual is not an independent validation of physical accuracy.

After fitting, **Show undistorted selected image** previews distortion removal
using the fitted K. **Export JSON / XML** writes either:

- JSON: camera/mode/lens identity, board dimensions, K/D, diagnostics and per-view
  residuals/extrinsics. Every fit also saves `intrinsics.json` in its session.
- OpenCV XML: K/D compatible with the older rectification loader, image size and
  serial; a companion JSON preserves all metadata. XML contains no robot frame.

New session images/manifests/results live beneath:

```text
~/.local/share/pickplace-workbench/camera-calibration/
```

`XDG_DATA_HOME` changes this root. `session.json` stores all refined corner
coordinates, camera settings per image, enabled/excluded views and the current
result. Configuration/lens controls are fixed after the first accepted view.
Samples with different camera serials, full image sizes, native ROI/binning/zoom,
resolution presets, mirror states or lens setup descriptions cannot be mixed.
A calibration is valid only for that recorded camera mode and physical lens setup;
do not apply full-resolution K unchanged to an ROI/binned mode. Externally changed
focus cannot be detected automatically; the operator must confirm it.

## What this does and does not establish

Intrinsics characterize **the lens and sensor projection**. No assumption is made
that the camera optical axis and TCP +Z coincide or are parallel. The mounting
photo shows a camera offset from the diamond tip; an image cannot measure that
rigid transform accurately. The existing four-point TCP calibration is preserved.

This feature does not estimate a robot-base workpiece plane, command TCP alignment,
or change robot calibration. Those later operations need hand–eye calibration
(camera-to-flange/TCP transform) plus the robot pose at observation time. A simple
unmarked chessboard may also have a 180° coordinate-index ambiguity relevant to
future workpiece origins/X directions; this does not prevent intrinsic fitting.
Pixel residuals must not be reported as TCP millimetre positioning accuracy.

## 简体中文步骤

1. 启动器选择 **Camera preview / 相机预览**，点击启动。在预览窗口点击
   **Intrinsic calibration / 相机内参标定**。
2. 棋盘填写 **11列 × 8行内角点，3 mm 方格边长**。先固定镜头对焦/光圈、分辨率及
   相机模式，填写镜头状态并勾选确认。
3. 每次保持完整棋盘清晰可见，点击 **采集当前视图**。采集会保存原始图像和细化角点。
4. 采集 **20–30张**：覆盖中心、边缘、角落，改变距离，并绕棋盘两个方向倾斜。
   建议固定机器人、手动移动平整的棋盘。仅平移或在平面内旋转不够。
5. 最少10张才允许计算；检查全局和每张图的重投影误差、覆盖率和警告。
   取消勾选坏样本后重新计算。点击导出保存 JSON 或 XML。
6. 自动恢复上次采集；镜头、分辨率、ROI、合并像素或镜像设置改变时必须新建。
   已采集的第一张实际图像保留在上面的 `ge502m-11x8-3mm-initial` 目录中，尚未完成标定。

内参只描述相机投影，不会把相机光轴当成 TCP +Z，不会移动机器人或替换四点 TCP 标定。
下一步手眼标定才会确定相机与机器人之间的刚体变换。

## Verification

Synthetic tests recover known K/distortion from varied camera poses, reject
front-parallel degenerate views, check duplicates and geometry/lens mismatches,
and verify saved sessions and JSON/XML exports. Qt tests cover native capture,
background detection, persistence/recovery, failed captures and safe close during
calculation. The actual camera was used to capture/refine an 88-corner native
2592 × 1944 view. A complete physical calibration awaits the operator's varied
views; no fictitious intrinsics are generated from the single test view.

```bash
QT_QPA_PLATFORM=offscreen .venv-camera/bin/python -m pytest \
  tests/test_intrinsics.py tests/test_intrinsic_window.py tests/test_camera_window.py -q
```
