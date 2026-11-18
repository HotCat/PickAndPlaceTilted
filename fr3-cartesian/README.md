# FR3 Cartesian Studio

[简体中文使用说明](README.zh-CN.md) · [Production development plan / 产线开发计划](docs/production-plan.md) · [Architecture and maintenance / 架构与维护](docs/architecture.md)

A PySide6 prototype for using a FAIRINO FR3 as an XYZ positioning robot: guided TCP calibration, multiple workpiece planes, joint teaching controls, and Cartesian arrows with a fixed tool orientation. First launch selects Real FR3 but stays disconnected; the last selected backend is remembered. No robot connection or movement starts automatically.

## Language

Use **Language / 语言** at the top right to switch between **English** and **简体中文** immediately. The choice is saved per OS user using Qt settings (`~/.config/FR3Cartesian/Studio.conf` on this Linux desktop). Switching updates controls, guidance, status, calibration results, file dialogs and existing operator messages without rebuilding the forms or issuing a robot command. Calibration values, plane names entered by the user, paths, error codes and physical units are preserved. Original SDK/OS diagnostic details remain in their source language.

You can also choose the startup language without changing the saved preference:

```bash
.venv/bin/python -m fr3_cartesian --language en
.venv/bin/python -m fr3_cartesian --language zh_CN
```

Translations live in `fr3_cartesian/locales/zh_CN.json`. English source templates and named parameters travel separately between the robot worker and UI, so errors can be displayed in either language without changing robot behavior or numerical precision. This is the localization foundation for subsequent production-line features; production milestones are described in the linked plan.

## Run

This application lives in `fr3-cartesian/` inside PickAndPlaceTilted. It has its own Python package and environment, separate from the camera workbenches at the repository root. Run all commands below from this directory. After creating `.venv` with the setup commands below:

```bash
source .venv/bin/activate
python -m fr3_cartesian
```

Or, without activating:

```bash
.venv/bin/python -m fr3_cartesian
```

To recreate the installed environment on Ubuntu, use the system Python 3.12 explicitly:

```bash
/usr/bin/python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements.lock
.venv/bin/python -m pip install -e .
.venv/bin/python tools/install_sdk.py
```

The launcher also supports `fr3-cartesian` after installing the project. `QT_QPA_PLATFORM=offscreen` is useful for headless tests; omit it for normal desktop use. On Linux, Qt's xcb platform may require your distribution's `libxcb-cursor0` package.

For another Python version (3.10+), install with `.venv/bin/python -m pip install -e '.[dev]'` instead of the lock file so pip can choose compatible wheels.

### Linux startup crash with Anaconda

Create this GUI environment with Ubuntu's `/usr/bin/python3.12`, even when your shell has Conda `base` activated. A virtual environment made by Anaconda Python retains its interpreter's native-library search paths. A previous installation mixed Anaconda's `libxkbcommon.so.0` with Ubuntu's `libxkbcommon-x11.so.0`, causing a segmentation fault in Qt's `QXkbCommon::verifyHasLatinLayout` during `QApplication` construction.

Recreate the project environment with system Python rather than copying an existing Conda environment. Virtual environments and native SDK files are not included in Git.

## Try the complete simulator workflow

1. Select **Simulator** and click **Connect** (first launch defaults to Real FR3, disconnected).
2. In **Tool calibration**, click **Simulator: next contact** four times, then **Calculate TCP**. The known demo TCP is `[0, 0, 120]` mm relative to the flange. Click **Apply TCP to this slot**.
3. In **Workpiece planes**, click **Simulator: next plane point** three times, then **Calculate & add plane**. These points define a tilted plane so its Z axis differs from base Z.
4. In **XYZ controls**, select the plane, click **Lock current tool orientation**, inspect the limits and axes, check the verification box, and enable Cartesian motion.
5. Click **+Z** to move 1 mm along the plane normal. Other arrows move along its X/Y axes. For a specific location, enter XYZ in plane millimetres and click **Move to XYZ**.

Joint and base-frame Cartesian jog controls use press-and-hold behavior: press `+` / `−` to start, release to send the matching `StopJOG`, and a configurable travel cap stops the motion if the button remains held. Joint travel is adjustable from 0.1° to 10°; base translation from 0.1 to 50 mm; base rotation from 0.1° to 10°. Separate joint and TCP speed sliders and percentage fields set jog speed from 1% to 10% (default 3%). The base-frame panel exposes X/Y/Z and RX/RY/RZ using FAIRINO `ref=2`; it is independent of workplane calibration. The XYZ tab remains calibrated workplane positioning with locked tool orientation.

## Connect the FR3

The development controller reports **V3.8.25-QX** (internal software **v3.8.5.1**). The installer selects official SDK **v2.1.5**, tagged for controller **3.8.5**, pinned to commit `f2d762e37f0e9c18d63340eba5972bdb81159276`. Its checksum and download URL are in `tools/install_sdk.py`. The installer preserves a replaced SDK as `vendor/fairino/Robot.py.previous`.

This SDK sends `MoveL` as one flat XML-RPC array. The previously installed 3.8.2 SDK sent 13 arguments, producing controller fault `-502` on plane moves. After updating, close all application windows and restart with **SDK folder** set to this project's `vendor` directory. The connection diagnostic logs the loaded SDK path and version. Offline tests verify the serialized motion request and ensure failed commands are never replayed. Motion with the replacement SDK has not been tested on hardware; the QX variant has no exact public SDK tag. If FAIRINO supplies a dedicated package, choose its parent as **SDK folder**.

1. Connect the computer to the robot network. The default robot IP is `192.168.58.2`. SDK 3.8.5 uses TCP 20003 for XML-RPC and TCP 20004 for feedback.
2. In the robot WebApp/pendant, configure the real tool payload mass and center of gravity, mounting, joint limits and collision protection. Select an existing correct payload profile. Stop any running Lua program. Prepare the robot in manual mode and enable it through the controller. This app does not change those settings or clear faults automatically.
3. Select **Real FR3**, enter its IP and SDK folder (default: this project's `vendor` directory), and click **Connect**. You can change the backend while connected to the simulator; doing so disconnects that session. Connecting reads state only. The tool and workpiece slot numbers are shown in the sidebar.
4. Teach the tool and workplanes below. For Cartesian motion, the controller must have the applied tool selected. The app checks the active slot and TCP offset before every Cartesian command. In **XYZ controls**, click **Use base workpiece 0** before enabling Cartesian motion, or select it through WebUI. The app transforms plane targets into robot base and sends each move with `user=0`; it requires the selected controller frame to match. Selecting another frame produces a local refusal without sending motion or closing the connection.

The controller's normal operating interlocks still apply. The software stop is not a safety-rated emergency stop and cannot guarantee delivery after a network failure; keep the physical stop accessible and verify clearances with small steps. Software travel bounds cover the TCP, not the entire arm or tool envelope. FR3 reach, joint limits, singularities and collisions still constrain movement.

For joint or base-frame teaching, check **Enable teaching jog controls** and hold an axis button; release it to stop. TCP/plane calibration and workpiece 0 are not prerequisites for these jog modes. Live joint angles and a readiness message appear beside the controls. Base jog follows the controller's base axes, not the calibrated workplane axes. The SDK command is `StartJOG(ref=0)` for joints or `StartJOG(ref=2)` for base Cartesian axes, paired with `StopJOG(ref=1)` or `StopJOG(ref=3)` on release. Background reads cannot cause a release to be dropped: a queued but undispatched jog is canceled, and a dispatched jog is followed by its stop command.

If connecting reports `Connection refused`, the controller port is unavailable or the network is rejecting connections. The app identifies command port 20003 and feedback port 20004 separately. Check the controller services and network, then click Connect again. An initial connection failure sends no stop commands; failures after a session is established still request a stop and invalidate the session.

FAIRINO return code **74** means **line instruction point error**. Controller fault **1/2** is **line target point error (including tool mismatch)**. Review/reset that fault in the official WebUI, select the calibrated tool and base workpiece 0, then reconnect. Do not recalibrate solely because of this error. A November 18 read-only check found valid IK/FK solutions for all six 1 mm steps around the current pose on the saved Pick plane, while workpiece 1 was selected. This does not verify a physical path or prove the frame mismatch is the sole cause. An actual MoveL rejection or uncertain communication failure still stops and invalidates the session; it is never automatically retried.

## Tool calibration

Use a repeatable contact point on the actual gripper/tool (the intended pick center), plus a rigid reference pin or other fixed target. Touch exactly the same target from at least four substantially different wrist orientations, tilting around two axes. Avoid sliding the target or changing the tool between samples.

Move with the pendant, drag teaching, or the app's separately enabled joint controls. **Exit drag mode and wait for the robot to stop before each capture.** Capture flange poses, calculate the TCP, and review contact RMS, maximum error and conditioning. The calculation rejects insufficient orientation variation and contacts exceeding the configured maximum residual (default 0.5 mm). More than four contacts are supported.

This is a pivot least-squares calibration of the tool-tip **translation**:

```text
R_flange[i] × TCP_offset + flange_position[i] = fixed_reference
```

Tool rotation is set to `[0, 0, 0]`, preserving flange-aligned tool axes. A pivot alone cannot determine tool-axis orientation. Full six-point tool-axis calibration is outside this prototype; it is not needed for the plane-based XYZ controls, which lock the existing TCP orientation in base coordinates.

Choose a tool slot (1–14), physical tool ID and an existing payload profile number. **Apply TCP** calls the official SDK's `SetToolList` and `SetToolCoord`; it updates that controller slot. Reusing a slot replaces its previous definition. Changing tool samples invalidates all workplanes because their contact positions depend on the tool offset.

## Workpiece planes and motion

With the calibrated TCP applied, capture:

1. **O**: desired workpiece origin.
2. **X**: another point along its positive X direction, at least 10 mm away.
3. **Y**: a point on the positive Y side of the XY plane, also at least 10 mm from O.

The app orthogonalizes these directions and uses a right-handed basis, with positive Z given by `X × Y`. Confirm positive Z points away from the surface before enabling movement. Nearly collinear reference points are rejected. Repeat with another name for a separate pick/place plane.

Workplanes are saved locally. Controller workpiece slots are updated only through the explicit apply action below. The transform is:

```text
base_position = plane_origin + plane_rotation × plane_XYZ
```

**WebUI Wobj jogging uses the controller's applied workpiece frame.** `Tool1, Wobj1` identifies tool slot 1 and workpiece slot 1. Saving a local plane or applying TCP alone does not upload the plane. To synchronize:

1. Connect and verify/apply the calibrated TCP. In **Workpiece planes**, select the saved plane.
2. Choose a **Target controller Wobj slot** (1–14). Review the preview: the plane's base-frame origin and RX/RY/RZ define the controller frame; these angles differ from the locked TCP orientation.
3. Check the replacement acknowledgment above **Apply plane to controller**, then click the button. When disabled, the guidance immediately above the button and its tooltip explain the first unmet requirement (connection, selected plane, fresh feedback, fault/stop, active program, verified TCP, or replacement acknowledgment). This replaces and selects that Wobj slot using `SetWObjList` and `SetWObjCoord`, with reference frame 0 (base). The robot must be stationary with no active program or fault; the app sends no motion or automatic fault reset.
4. Wait for the verified message. The app checks the active Wobj number and reads back its translation and rotation. It does not retry a failed write. In WebUI select **Wobj** jog with the same tool/Wobj numbers.

The target slot preference is saved with calibration; the acknowledgment and verification receipt are not restored across connections. Startup, connection and local save/load never upload a frame. This feature has passed simulator and official-SDK XML-RPC serialization tests; its physical jog direction and persistence through a controller restart still require hardware verification.

For this app's **XYZ controls**, click **Use base workpiece 0**, then lock orientation and verify limits again. Those arrows continue to transform the selected local plane into base coordinates. Selecting base does not erase the taught Wobj slot. The workpiece-tab preview shows the **frame definition**; the six numbers shown while WebUI is jogging are the **live TCP pose** and are not by themselves the stored Wobj definition.

See [WebUI Wobj jogging](https://fairino-doc-en.readthedocs.io/latest/CobotsManual/teaching_pendant_software.html) and [workpiece frame SDK APIs](https://fairino-doc-en.readthedocs.io/latest/SDKManual/PythonRobotCommonSettings.html).

On the tested FR3, both TCP and flange feedback are relative to the active Wobj. The app reads the active frame around a live flange-pose query, checks that it stayed unchanged, and converts the complete flange pose into robot-base coordinates before applying the TCP offset. Captures therefore retain one common base reference even when the selected Wobj changes. Commands use the transformed base target with `MoveL(tool=selected_slot, user=0)`. Select base workpiece 0 first; a matching tool slot and TCP offset alone are insufficient. **Use base workpiece 0** explicitly applies an identity frame to slot 0 with `SetWObjCoord(0, [0,0,0,0,0,0], 0)`, then verifies the active slot and offset. It sends no motion and leaves Wobj slots 1–14 intact. Switching this way disarms motion and requires a fresh orientation lock and axes/limits verification. The app never selects a frame at startup or on connection, and this button does not upload a saved plane. Each plane move also checks that the active base offset is zero. The locked base-frame orientation is copied to every target. Moving a joint or switching planes requires locking orientation again.

Set per-axis travel bounds before enabling motion. Both the starting TCP and the destination must be inside the same box, so a straight segment stays inside it. Single absolute moves are limited to 100 mm. Arrow steps are 0.1–10 mm; hold mode always uses 1 mm. Speed is a controller **percentage**, default 3%, capped at 10%; it is not mm/s. There is no automatic rotation, approach/retract path, gripper I/O or pick/place cycle in this prototype.

If **Move to XYZ (0, 0, 0)** reports that the current TCP is outside plane limits, the destination can be valid while the starting position is outside the permitted box. XYZ controls now show the current limit check beside the bounds, and the rejection reports the offending axis, current coordinate and allowed interval. Reposition near the intended workpiece using teaching controls, then lock orientation again. The origin is the saved workpiece origin, not the current TCP. Limits are never expanded automatically, and the 100 mm command cap still applies.

Only one command may be in flight. `MoveL` uses `blendR=0`, but the app submits no following point until live feedback confirms the target (within 0.02 mm) and stationary state. The worker monitors state during motion and stops on stale feedback or a fault. It never automatically retries uncertain motion commands. Requests have time limits; the isolated SDK process is discarded on an error/timeout to prevent late command replay. STOP/Esc and disconnect send `ImmStopJOG` and `StopMotion` through a separate bounded XML-RPC connection and invalidate the session. Losing focus during a held joint/base jog sends its normal `StopJOG` release while keeping the connection. Losing focus during an in-flight Cartesian move stops and disconnects; losing focus while idle leaves the connection open and disarms Cartesian controls.

If local target/tool validation rejects a move before any motion command is sent, the app disables Cartesian motion and explains the reason while keeping the connection open. Faults, stale feedback, communication failures, and errors after submitting a move still use the stop/disconnect path. Rejected commands are never retried automatically.

## Repairing planes captured by older versions

Older versions incorrectly treated Wobj-relative flange feedback as robot-base feedback. If points were captured with a nonzero Wobj selected, uploading the resulting local plane as a base definition applied the reference transform twice. This explains the reported large WebUI XYZ offset near the taught origin and the wrong Wobj jogging directions. The read-only diagnosis and its limitations are recorded in [the coordinate-frame incident note](docs/coordinate-frame-fix.md).

Because the original capture frame of each point is uncertain, this update preserves the old data but blocks those planes. Reteach the plane; the existing TCP offset can be kept and verified against the controller.

1. Close all old app windows and start this version. Connect and verify the saved TCP; use **Apply saved TCP** only if it does not match.
2. In **XYZ controls**, click **Use base workpiece 0**. This explicit selection sends no motion. Corrected captures also support other active Wobj frames, but using base 0 makes the teaching reference clear.
3. In **Workpiece planes**, select the old plane and click **Remove plane**. Click **Clear points** to discard its old draft. These actions leave the TCP calibration intact.
4. Teach fresh O, X and Y points while stationary, then **Calculate & add plane**. Choose their order so +Z points away from the surface. Review the new base-coordinate preview.
5. To use WebUI Wobj jogging, explicitly apply the new plane to the intended controller slot, such as Wobj1. The existing incorrect controller definition remains until you replace it. At taught O, WebUI XYZ should be near zero with the correct tool and Wobj selected. RX/RY/RZ describe the tool's orientation relative to that plane and need not be zero.
6. To use this app's XYZ controls, select **Use base workpiece 0** again, lock the current orientation, and verify directions and travel limits before enabling motion.

## Save and load

The app automatically saves changes to `calibration-session.json` in this application directory (`fr3-cartesian/`) and restores them on startup. This includes raw TCP contacts and the calculated TCP, all workplane names and teaching points, unfinished plane points and their name, the selected plane, XYZ travel bounds, step, speed, hold mode, target XYZ, and the last orientation reference. Changes save after a short pause and are flushed on close. Files are replaced atomically so an interrupted write does not truncate the previous save. Errors appear in the log.

On the first updated launch, if no autosave exists, the app imports the existing project `calibration.json` without modifying that original file. **Save calibration…** and **Load calibration…** remain available for named backups and exchanging setups. New exports use schema 3 and record the coordinate reference of TCP samples, plane points and unfinished captures alongside the XYZ settings; schema 1 and 2 files remain readable. Old real-robot measurements without reference metadata are retained as `legacy_workpiece_unknown`. Those planes cannot be uploaded or used for motion, and new captures cannot be mixed into their old draft points. Derived TCP and plane transforms are recalculated from their raw measurements on load.

Startup restoration does not connect or send controller commands. After you connect (or load a file while connected), the app reads the active tool slot and all six TCP offsets and compares them with the saved calibration (0.001 mm/degree tolerance). A match confirms the TCP without writing controller settings. Otherwise, click **Apply saved TCP** in **XYZ controls**, or apply it in Tool calibration; this explicitly updates the saved tool slot and waits for confirmation. The XYZ panel shows the verification status. Changes to the active slot or offset revoke confirmation.

A saved orientation is displayed as a reference, not an active orientation lock. After TCP confirmation, lock the current orientation and confirm axes/limits before enabling motion. Simulator calibration cannot be applied to a real robot. A saved file cannot detect a moved fixture or replaced tool.

The application focuses on TCP calibration, workpiece planes, on-screen teaching jogs and calibrated XYZ positioning. The joystick pendant is removed from the UI. Base-frame jogs have their own **TCP jog speed** slider and percentage field (1–10%, default 3%), independent of joint jog speed. Jog speeds and travel caps persist with the calibration session.

## Verification

```bash
.venv/bin/python -m pytest -q
QT_QPA_PLATFORM=offscreen .venv/bin/python -m fr3_cartesian --smoke-test
.venv/bin/python -m fr3_cartesian --smoke-test
```

The last smoke test must run in a desktop session: it exercises the actual X11/Wayland platform and keyboard libraries. Headless tests alone cannot detect native desktop-library conflicts.

Tests cover pivot recovery and degeneracy, tilted-frame transforms, orientation preservation, travel limits, bounded joint/base jog starts and release stops in simulation, SDK error handling without replay, jog-stop arbitration during feedback polling, backend switching, jog readiness, and a full bilingual Qt/process simulator workflow including save/load and stop. Read-only hardware feedback has been verified; hardware calibration accuracy, full `QX` motion compatibility, physical jog behavior and physical stopping remain unverified.

## Vendor references

- [Python manual](https://fairino-doc-en.readthedocs.io/latest/SDKManual/python_intro.html)
- [Motion APIs](https://fairino-doc-en.readthedocs.io/latest/SDKManual/PythonRobotMovement.html)
- [Official SDK v2.1.5 / robot 3.8.5](https://github.com/FAIR-INNOVATION/fairino-python-sdk/tree/v2.1.5_robot3.8.5)

The current manual describes newer firmware too. The installed pinned SDK source, not a guessed signature from the latest documentation, determines the adapter's actual calls.

Error definitions: [SDK error table](https://github.com/FAIR-INNOVATION/FAIR-documentation/blob/main/source/SDKManual/errcode.rst) and [controller fault table](https://github.com/FAIR-INNOVATION/FAIR-documentation/blob/main/source/CobotsManual/appendix.rst).

## Repository contents

Only source, documentation, tests and dependency pins are tracked. The SDK installer downloads the unmodified vendor module with a SHA-256 check; `vendor/` remains ignored. Virtual environments, calibration sessions, legacy cell calibration, HID profiles and local diagnostics are excluded. Back up existing cell measurements separately and use **Load calibration…** to import them deliberately. Test inputs are synthetic; no commissioning data is required to run the suite. The retained `pendant.py`, `joystick.py` and `velocity.py` modules are historical experiments, not an enabled operator interface.
