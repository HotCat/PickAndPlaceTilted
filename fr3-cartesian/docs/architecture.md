# Architecture and maintenance / 架构与维护

## Scope inside PickAndPlaceTilted

This subproject is a standalone commissioning application. The repository root
contains camera acquisition, rectification and PCB/CAD registration tools; none
of those tools currently calls this robot worker. Keep the FR3 virtual environment
inside `fr3-cartesian/.venv/` and install the package in editable mode. The UI
resolves `vendor/` and calibration files beside the package, independently of the
shell's current directory. These paths assume the editable source layout; a wheel
installed into a read-only system location is not a supported deployment yet.

本子项目用于机器人调试，与仓库中的相机 / CAD 工具分别运行，当前没有自动运动联动。
请在 `fr3-cartesian/` 中创建 `.venv`，并使用可编辑安装。SDK 与标定文件按源码目录
定位，不按终端当前目录定位。当前尚未支持安装到只读系统目录的正式部署方式。

## Process ownership and command lifecycle

| File | Responsibility and maintenance rule |
| --- | --- |
| `app.py` | Widgets, operator intent and calibration workflow. Never import the vendor SDK here. |
| `client.py` | JSON-lines bridge to a `QProcess` using the same Python interpreter. Owns request IDs, command arbitration, deadlines and stop-process lifetime. |
| `worker.py` | Sole owner of the SDK session; validates fresh controller state and executes each command once. Includes deterministic simulation. |
| `feedback.py` | Incremental checksum-validated TCP state parser and receive timestamps. A socket read is not assumed to contain a complete packet. |
| `geometry.py` | Pure pivot calibration, rigid transforms and TCP travel bounds; no Qt widgets or robot I/O. |
| `persistence.py` | Validate complete documents before applying UI state, recompute derived transforms and atomically replace saved files. |
| `messages.py`, `i18n.py`, `locales/` | Carry message templates and named values separately; translate only at the presentation boundary. |

The worker's standard output is a machine protocol; SDK diagnostic output belongs
on standard error. Do not add ordinary `print()` calls to the protocol path.
Only one normal request is in flight. A pending telemetry read can queue one user
command. Releasing an on-screen jog cancels an unsent start or queues its stop
behind a dispatched start/read; it must not discard release because the worker is
busy. Add regression coverage in `test_client.py` when changing that arbitration.

`NoRetryTransport` deliberately bypasses XML-RPC's normal reconnect-and-retry
behavior. A request may have reached the controller even when its reply is lost.
Retrying it could repeat motion or a controller write. Motion/communication errors
invalidate the session, attempt stop through an independent bounded process, and
require an explicit reconnect. Preflight rejections that sent no motion keep the
connection and disarm Cartesian motion. Do not collapse these outcomes into a
single generic retry path.

界面与 SDK 通过独立进程隔离，SDK 仅由 worker 持有。标准输出是 JSON 协议，诊断
信息走标准错误。普通请求保持单条在途；点动松开事件必须取消未发送的启动，或排队
发送停止。通信错误不能证明指令未执行，因此运动和控制器写入不得自动重发。

The feedback adapter timestamps complete validated packets with a monotonic clock.
Keep the last valid packet on partial/corrupt input. Socket timeouts alone do not
mean a disconnect; freshness checks determine whether motion is allowed. The SDK's
`reconnect_flag` means a reconnect loop is in progress and some vendor methods wait
on it indefinitely. Permanent receive errors therefore use a separate application
field so bounded stop attempts are still possible. See `test_feedback.py`.

Plane arrows tag each `MoveL` with a unique stroke token. Release cancels a queued,
unsent move locally or sends a `cancel_move` message outside the normal request
queue. `CommandInput` polls for matching cancellation before dispatch and while
waiting for motion completion; other commands stay queued and SDK calls remain
serialized. The worker sends `StopMotion` once, waits for stationary and program-
stopped feedback, and only then completes the request. Late tokens cannot stop a
new stroke or an absolute target move. The repeat timer is owned and canceled on
release; no delayed callback can restart a released stroke. The simulator advances
these tagged moves progressively so tests can verify stopping short of the target.

A release not acknowledged within two seconds discards the SDK owner and uses the
independent stop channel. This bounds a blocked SDK/read call; it does not promise
a physical stop within that interval. Normal confirmed release keeps TCP/plane
verification and the connection; focus loss also disarms Cartesian controls.

平面方向键的每次步进使用唯一标识。松开时取消未发送的步进，或直接发送停止请求，
不能排在正在执行的 MoveL 后面。后台串行调用 StopMotion，并确认机器人和程序停止；
旧标识不能影响新的步进或绝对定位。停止超时走独立停止通道并断开，正常确认后保留连接。

STOP/Esc and uncertain motion failures use the independent stop channel. Focus loss
during a held native jog sends its normal release stop. Focus loss while idle
disarms Cartesian controls without closing the connection. These are software
controls, not a replacement for the cell's physical safety system.

## Coordinate conventions

All lengths are **millimetres**, and Euler angles are **degrees**. Rotation matrices
use `Rz(rz) @ Ry(ry) @ Rx(rx)` and act on column vectors. A plane basis stores its
positive X, Y and Z unit vectors as **columns expressed in robot base**. O/X/Y
teaching orthogonalizes the Y direction and chooses right-handed `Z = X × Y`.

```text
T_base_flange = T_base_wobj × T_wobj_flange
T_base_tcp    = T_base_flange × T_flange_tcp
p_base_target = plane_origin + plane_basis × p_plane_target
```

The development controller exposes flange/TCP poses relative to its selected
workpiece frame. Read the tool/Wobj references before and after a live flange
query; reject unsettled references. Compose the complete pose into base before
using it as a calibration sample. Adding a translation alone fails when the Wobj
is rotated. `test_sdk_wire.py` covers this with synthetic, analytically known data.

Pivot calibration solves `R_i × tool_offset + flange_position_i = fixed_contact`.
It determines TCP translation, not tool-axis rotation; tool rotation remains zero.
Changing TCP samples invalidates plane geometry because the contact point changes.
Fit residuals describe the calibration contacts; acceptance over the actual working
volume requires independent repeated contacts with different wrist orientations.

Two different controller operations are intentional:

1. **XYZ controls:** transform the selected local plane into base, retain the locked
   TCP attitude in base, and send `MoveL(tool=slot, user=0)`. Require identity Wobj0
   and matching active tool/offset. No implicit frame selection before moving.
2. **Apply plane to controller:** explicitly replace/select the acknowledged Wobj
   slot 1–14 for official WebUI Wobj jogging, then read back its definition. This
   sends no motion and does not change the app's local plane definition. Select
   base Wobj0 again before using this app's XYZ controls.

基坐标、工件坐标和工具偏移必须明确区分。本程序的平面 XYZ 目标先转换到基坐标，
再使用 Wobj0 运动；“应用平面到控制器”则用于同步 WebUI 的 Wobj 点动。保存本地
平面不等于上传控制器坐标系。标定时必须转换完整位置和姿态，不能仅叠加平移。

Native teaching uses `StartJOG(ref=0)` for joints and `ref=2` for base Cartesian
axes, paired with `StopJOG(ref=1)` and `ref=3` respectively. Joint speed and base
TCP jog speed are separate **controller percentages**, 1–10%, default 3%. The TCP
setting also applies to RX/RY/RZ; it is not a calibrated physical mm/s command.
Travel caps stop a held jog even if the operator does not release its button.

## Persistence and migration

Schema 3 saves raw TCP contacts, plane points and their reference provenance,
drafts, selections, limits, targets, orientation reference, jog speeds and travel
caps. Derived results are recomputed when loading. `QSaveFile` prevents a partial
write from truncating the previous session. Schema 1/2 imports are supported;
real-robot points lacking known references are marked `legacy_workpiece_unknown`
and blocked for plane motion/upload until reteaching. Do not silently relabel
unknown measurements as base coordinates.

Connection, teaching enable, Cartesian enable, replacement acknowledgment and an
active orientation lock are not restored as permissions. A saved orientation is
only a reference. Verify the TCP on each connection and explicitly lock orientation
and confirm directions/limits before plane motion. UI preferences use Qt settings;
switching language never issues robot commands.

标定文件保存原始测量与坐标来源；加载时重算结果。旧实机数据若缺少坐标来源，必须
保留为未知并重新示教。重启后不恢复连接、运动使能或有效姿态锁。现场标定和本机
配置均被 Git 忽略，应单独备份，不能把他人的测试标定直接当作本机有效数据。

## SDK compatibility and reproducible checks

`tools/install_sdk.py` pins official SDK v2.1.5 / robot 3.8.5 at commit
`f2d762e37f0e9c18d63340eba5972bdb81159276` and checks its SHA-256 before writing
the unmodified module to ignored `vendor/`. The development FR3 reports
V3.8.25-QX (internal v3.8.5.1); the public tag is not an exact QX-specific package.
Do not update to the latest SDK merely because online documentation changes.

The pinned SDK serializes `MoveL` as **one flat XML-RPC array**. An older SDK's
13-argument form failed on this controller with fault -502. Wire tests bypass
`RPC.__init__`, replace the transport with canned responses and inspect the exact
serialized request. They open no sockets. When changing SDK versions, run these
tests with the new source and review the wire layout, state structure and reference
conventions before supervised hardware commissioning.

From `fr3-cartesian/`, after the documented environment setup:

```bash
.venv/bin/python tools/install_sdk.py
QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q
QT_QPA_PLATFORM=offscreen .venv/bin/python -m fr3_cartesian --smoke-test --language en
QT_QPA_PLATFORM=offscreen .venv/bin/python -m fr3_cartesian --smoke-test --language zh_CN
```

Without the SDK, serialization tests intentionally skip; a passing run with skipped
wire tests does not validate SDK compatibility. A desktop smoke run without
`QT_QPA_PLATFORM=offscreen` is also needed when changing the Python/Qt environment,
because headless rendering cannot detect native X11/Wayland library conflicts.
Tests require no robot or HID device. Simulator motion does not establish hardware
accuracy, path clearance, stopping distance or timing under production load.

## Direction for task-specific motion

The enabled UI has three tabs: tool calibration, workpiece planes and XYZ controls.
`pendant.py`, `joystick.py`, `velocity.py` and associated tools/tests are retained
historical experiments; `app.py` neither imports the pendant nor polls HID input.
Python position accumulation over XML-RPC did not provide the desired LinuxCNC-like
joystick experience. Do not reintroduce it as a production pendant by exposing a
checkbox. Historical behavior is described in `joystick-pendant.md`.

For vision-guided pick/place, introduce a recipe/state-machine layer between UI
intent and the worker. Camera detections need an explicit, verified camera-to-plane
registration with defined units, robot/tool/fixture identity and freshness. A pixel
coordinate or CAD similarity fit is not yet a robot target. A cycle should model
approach, descend, grip verification, retract, transfer, release and recovery with
bounded I/O waits. Do not wire camera callbacks directly to `MoveL` or automatically
resume a cycle after a process restart or uncertain motion/grip result. See the
separate production plan for acceptance milestones.

后续应围绕取放配方和状态机开发，而不是继续扩展摇杆。相机像素 / CAD 配准结果需要
经过已验证的相机到工件平面标定才能生成机器人目标。自动循环需明确接近、下降、
夹取确认、抬升、转移、释放及异常恢复；禁止把相机回调直接接到运动指令。


## Static pick/place coordinate recipe

`tasks.py` is a pure helper: it maps each row-major plane XYZ source through the
saved plane transform to base, gives it the current explicitly locked orientation,
and emits named, absolute base TCP poses. A fixed six-axis base destination and an
independent operator-authored route for every array cell define the transfer. The
next cell's route is reversed between the destination and that following source.
Legacy sessions with one shared route are migrated to independent copies. It adds
no hidden lift, gripper state, vision correction or collision planner.

The UI review gate checks each consecutive segment against the same workplane TCP
limits, current start pose and 100 mm per-segment `MoveL` cap. Each target then goes
through the worker's existing TCP/tool/Wobj/fresh-state preflight. The worker and
client remain single-flight; GUI completion advances one segment at a time only
after arrival/stationary feedback. A failed or rejected point clears the remaining
sequence; operator cancellation uses STOP and invalidates the session. There is no
automatic resume after a failed point or restart. Recipe coordinates are persisted,
but the review acknowledgment, start permission and active sequence are not.

The table is a manually calibrated static source list. It is not a vision interface:
future localization must transform measured workpiece coordinates into this same
plane frame and should carry freshness and confidence before a cycle can consume it.
The UI exposes the ordered base poses for review, but operators must validate the
full physical arm path. Straight segments between points can still intersect a
fixture even if both endpoints are inside the TCP box.
