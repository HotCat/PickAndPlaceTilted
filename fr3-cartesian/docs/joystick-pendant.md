> Historical experiment: the joystick pendant has been removed from the application UI. The instructions below describe the retired prototype. Use the on-screen teaching jogs and calibrated XYZ controls.
> 历史实验：摇杆示教页已从程序界面移除。以下内容描述已退役的原型，请使用界面示教点动和已标定 XYZ 控制。

# PXN 2113 Pro teaching pendant / 莱仕达 2113 Pro 示教摇杆

The connected device was identified on 2026-11-18 as **PC Game Controller**, USB **11ff:3331**, Linux HID driver **gembird**. It exposes four 0–255 analog axes (ABS_X, ABS_Y, ABS_RX, ABS_RY), a two-axis hat, and 12 buttons (288–299). Its stable input link is `/dev/input/by-id/usb-11ff_PC_Game_Controller-event-joystick`. The application discovers it by its USB IDs/name, so a changed `eventN` number does not invalidate the profile. No device-permission changes or robot commands were needed for identification.

2026 年 11 月 18 日检测到的设备为 **PC Game Controller**，USB **11ff:3331**，Linux HID 驱动为 **gembird**。设备有四个 0–255 模拟轴、两个方向帽轴和 12 个按钮（288–299）。程序按 USB 编号和名称识别设备，不依赖会变化的 `eventN` 编号。识别过程未更改设备权限，也未发送机器人指令。

## Start / 开始使用

1. Restart the application. Open **4 Joystick pendant**, select the device, and click **Open input monitor**. Opening input does not enable robot motion.
2. Move each physical control separately and press each button to identify it in the live raw-axis/button display. The default X/Y mapping is ABS_X/ABS_Y, with Y inverted. The confirmed PXN profile maps grip twist **ABS_RY (4)** to target axis 3 (Z in XYZ modes); the throttle **ABS_RX (3)** scales the TCP speed ceiling; it is independent of the centered motion axes. The trigger was physically confirmed as button **288**. Generic profiles leave input 3 disabled until identified. Select a different input or inversion in the third row if needed.
3. Leave the stick/twist released, press no buttons, and click **Measure released center (1 second)**. This saves a local profile. The saved PXN profile includes measured centers (X=137, Y=113, twist=123). It uses a 15% center dead zone, an additional 4% activation margin and a 1.5 response exponent for native jog/plane steps; Base XYZ velocity is linear after the dead zone. Recalibrate if you change the mapping or the stick's resting position changes. A throttle is not spring-centered; use the separate throttle mapping rather than assigning it to a motion axis.
4. Verify the default button assignments: **288 = held deadman trigger**, **289 = software STOP**, **290 = capture**. These are Linux button codes, not guaranteed printed labels on the joystick. Choose different codes in the panel and **Save joystick profile** if needed. Each code must have only one function.
5. Connect the simulator first. Choose J1/J2/J3, J4/J5/J6, base XYZ, or base RX/RY/RZ. Enable the existing **Teaching jog controls**, then **Enable joystick teaching**. Release all buttons and center the mapped axes. Hold the trigger and deflect the stick. Base XYZ accepts all three axes together; joint/rotation modes select the largest deflection. Return to center or release the trigger to stop.
6. Verify directions and button behavior in the simulator, then repeat the explicit setup with the real robot stationary, enabled in controller manual mode, with its Lua program stopped. Motion remains subject to controller faults and the existing readiness checks.

1. 重启程序，打开 **4 摇杆示教器**，选择设备并点击 **打开输入监视器**。打开输入不会启用机器人运动。
2. 分别移动每个实物控制轴、按下各按钮，在实时显示中确认对应代码。默认 ABS_X/ABS_Y 控制前两个目标轴，Y 反向。实物操作确认扭转轴为 **ABS_RY (4)**，已映射至第三目标轴（XYZ 模式中的 Z）；油门 **ABS_RX (3)** 调节 TCP 速度上限，不参与回中判断。扳机已实测确认为按钮 **288**。通用配置在识别前禁用第三输入轴；可按需要修改输入及反向设置。
3. 松开摇杆和扭转轴，不按任何按钮，点击 **测量松开后的中心（1 秒）**。测量后自动保存配置。已保存的 PXN 配置包含实测中心（X=137、Y=113、扭转=123），中心死区 15%，另加 4% 的启动余量；原生点动/平面步进响应指数为 1.5，基坐标 XYZ 速度在死区后呈线性响应。改变映射或中心位置后请重新测量。油门轴不会自动回中，请使用独立油门映射，不要将其分配为运动轴。
4. 确认默认按钮映射：**288 为按住使能扳机，289 为软件停止，290 为采集**。这些是 Linux 按钮代码，不保证与实物印刷编号一致。可在面板中修改后点击 **保存摇杆配置**，每个按钮只能分配一个功能。
5. 先连接仿真器，选择关节前三轴、后三轴、基坐标 XYZ 或旋转模式。启用原有 **示教点动控制**，再启用 **摇杆示教**。松开全部按钮并回中后，按住扳机、偏转摇杆。基坐标 XYZ 接受三轴同时输入；关节/旋转模式选择偏转最大的单轴。回中或松开扳机即请求停止。
6. 在仿真中验证方向和按钮，再对静止的实机明确启用。控制器需处于手动模式、机器人已使能且 Lua 程序停止；原有故障及就绪检查仍然有效。

## Motion behavior / 运动行为

**Base XYZ now uses simultaneous proportional velocity.** Dead-zone-corrected X, Y and twist inputs become one XYZ velocity vector; changing the stick changes velocity during the same stroke, including direction reversal. The response is linear after the dead zone. The vector magnitude is capped, so a full diagonal cannot exceed the configured TCP maximum. Velocity is integrated into an absolute TCP target in robot base, analogous to a simulated encoder, and sent through the pinned SDK's `ServoMoveStart` → `ServoCart(mode=0, cmdT=0.008)` → `StopMotion` / `ServoMoveEnd` sequence. Each target retains the initial tool orientation. This requires **base Wobj0 with zero offset**; use the explicit base selection button before enabling. TCP calibration is not required for base teaching, but the controller's current TCP definition must describe the installed tool.

**基坐标 XYZ 现使用三轴同时比例速度控制。** X、Y、扭转输入经过中心死区后组成 XYZ 速度向量；同一次操作中可实时改变速度与方向。死区后的响应为线性，斜向合成速度不会超过 TCP 最高速度。程序将速度积分为机器人基坐标下的绝对 TCP 目标，类似模拟编码器，并使用固定版本 SDK 的 `ServoMoveStart` → `ServoCart(mode=0, cmdT=0.008)` → `StopMotion` / `ServoMoveEnd`。每个目标均保持起始工具姿态。此模式要求 **基坐标工件 0 且偏移为零**，请先明确选择。基坐标示教不要求本程序完成 TCP 标定，但控制器当前 TCP 定义应对应所装工具。

**Throttle / 油门**: `Throttle axis for TCP speed limit` defaults to **ABS_RX (3)** in the measured PXN profile. The full hardware range maps to 0–100%, with a 2% endpoint margin to reach a stable true zero/full scale. `Invert throttle` swaps its ends. It is not centered or included in center calibration gating. The live readout shows throttle percentage, effective speed ceiling and commanded XYZ velocities. `Base XYZ maximum TCP speed` defaults to **10 mm/s**, adjustable from 0.1 to **25 mm/s**. The percentage setting is labeled **Joint / rotation / plane speed ceiling**; it does not set Base XYZ servo mm/s. The throttle also scales the speed of the next calibrated-plane step; a plane step already in flight retains its submitted speed.

实测 PXN 配置默认用 **ABS_RX (3)** 作为油门。其完整硬件范围映射至 0–100%，两端保留 2% 余量以稳定达到零和满量程；**油门反向** 可交换快慢两端。油门不要求回中，也不参与中心校准门控。实时显示油门百分比、有效速度上限和 XYZ 指令速度。**基坐标 XYZ 最高 TCP 速度** 默认 **10 mm/s**，可设为 0.1–**25 mm/s**。旧百分比点动速度不定义伺服 mm/s，继续约束原生关节/旋转点动。油门也会缩放下一条标定平面步进指令的速度；已提交的平面指令保持原速度。

**Shared travel / 共用行程**: The pendant's **Shared Base XYZ linear travel limit** and **Base frame Cartesian jog → Linear travel limit** are two views of the same setting, updated immediately in both directions. They cannot be edited during motion. For vector motion this caps the **total commanded path length per stroke**, including diagonals and reversals, rather than a separate distance for each axis. Reaching it ends servo motion; a held trigger cannot restart it. Recenter for a new stroke. This is a software command cap; following error and physical stopping distance still matter.

摇杆页的 **共用基坐标 XYZ 直线行程上限** 与 **基坐标系笛卡尔点动 → 直线行程上限** 是同一设置的两处显示，双向即时同步，运动期间禁止修改。向量运动中它约束一次操作的**总指令路径长度**（包含斜向与反向），不是给每个轴独立分配一份距离。达到上限后结束伺服运动，持续按住扳机不会重新启动，需回中后开始新操作。该值约束软件指令，仍需考虑跟随误差与实际停止距离。

**Joint and base rotation modes** retain native `StartJOG` / `StopJOG`, one axis at a time, with speed sampled at stroke start and the existing percentage caps. The response exponent only affects these native modes and the existing plane-step mode. They retain their separate joint/rotation travel caps.

**关节与基坐标旋转模式** 继续使用原生 `StartJOG` / `StopJOG`，每次单轴、速度在操作开始时取样，并使用原有百分比上限。响应指数仅影响这些原生模式及现有平面步进模式；关节与旋转保留各自的行程上限。

The servo loop runs in the SDK worker, not Qt. It sends no catch-up burst after a delayed cycle and never retries a motion write. GUI inputs expire after **150 ms** without a fresh heartbeat; a worker cycle delayed beyond **40 ms** stops the stream. Live packet checks cover controller readiness, tool/Wobj number changes, stale feedback, TCP following error (1 mm) and orientation changes. A TCP packet is checked against normalized live RPC feedback before starting; contradictory references block startup. Other controller commands are excluded until the stream ends. Initial inputs and late packets cannot revive an old session. STOP uses an independent bounded connection and also ends servo mode when it may be active.

伺服循环在 SDK 后台进程运行，不依赖 Qt 调度，不会在延迟后突发补发，也不重试运动写入。GUI 输入超过 **150 ms** 无新心跳即过期；后台周期延迟超过 **40 ms** 会停止流。每周期检查控制器状态、工具/Wobj 编号、反馈新鲜度、TCP 跟随误差（1 mm）和姿态变化。启动前先核对状态包与归一化实时 RPC 位姿，参考系不一致即禁止启动。运行期间排除其他控制器指令；旧输入和迟到的数据不能恢复已结束会话。STOP 使用独立有限时通信，并在可能处于伺服模式时结束该模式。

While a Base XYZ stroke is active, **centering with the trigger held commands zero velocity and keeps the same servo session**. Deflecting again resumes within its remaining path budget; crossing center or changing axes does not repeatedly start/end servo mode. Releasing the trigger or reaching the travel limit ends the session. The live display shows consumed stroke travel. A normal stop waits up to one second for stationary/stopped-program feedback before publishing completion, avoiding a false Lua-program blocker during controller settling. Failure to confirm settling still disables motion.

基坐标 XYZ 操作期间，**保持扳机按下并回中会将速度指令设为零，保留同一伺服会话**。再次偏转会在剩余路径预算内继续运动；经过中心或改变轴向不再反复启停伺服。松开扳机或达到行程上限才结束会话。实时界面显示已用行程。正常停止后，程序最多等待一秒，确认机器人静止且程序停止后才发布完成状态，避免将控制器停止过渡状态误判为 Lua 程序运行。无法确认停止仍会禁用运动。

Input uses a 20 ms Qt precise timer and sends each freshly sampled input before display work. The live monitor refreshes at most ten times per second. This reduces UI load without extending the 150 ms input expiry. `Joystick heartbeat expired` means the worker did not receive fresh input in time, rather than a travel-limit stop. Focus loss, delayed polling and input loss now log their cause. After suspension, release the trigger, center the axes and explicitly enable teaching again; old input never restarts motion.

输入使用 20 ms Qt 精确定时器，每次先发送新采样的输入，再更新显示；实时监视器最多每秒刷新十次，以降低界面负载，150 ms 输入过期阈值保持不变。`摇杆心跳过期` 表示后台未及时收到新输入，与达到行程上限不同。失焦、轮询延迟和输入丢失现在会记录原因。暂停后请松开扳机、将轴回中，并再次明确启用示教；旧输入不会重新启动运动。

The application installs a feedback reader around the pinned SDK without editing its vendor source. It preserves partial TCP headers, payloads and checksums across reads, validates the uint16 checksum, and retains the last valid state after corruption. A socket read timeout leaves the connection open; freshness checks still reject stale feedback (100 ms during servo motion). Freshness uses the last valid packet's receive time, independent of the wrapping frame counter. EOF/socket errors require explicit reconnection; the reader never reconnects by itself.

程序为固定版本 SDK 安装反馈读取适配器，不修改厂商源码。读取器可保留跨 TCP 分段的包头、载荷与校验和，验证 16 位校验和，并在坏包后保留最后一个有效状态。单次套接字读取超时不会关闭连接；反馈过期检查仍生效（伺服运动期间为 100 ms）。新鲜度使用最后有效包的接收时间，不依赖循环计数器。连接关闭或套接字错误后需明确重连，读取器不会自行重连。

On **V3.8.25-QX**, a recorded real-robot trace showed `program_state` and `robot_state` changing from 1 to 2 after three accepted `ServoCart` commands, with manual mode, enable and all fault/safety fields unchanged. The active stream accepts that running marker after its own first accepted servo command. It still requires a stationary robot and stopped program immediately before `ServoMoveStart`; paused/drag states, automatic mode, disable and fault/safety changes stop the stream. The pendant recognizes this worker-validated running state only while its own Base XYZ session is active. Readiness errors retain raw field values and the count of accepted servo writes.

**V3.8.25-QX** 的实机日志显示：接受三条 `ServoCart` 指令后，`program_state` 与 `robot_state` 从 1 变为 2，手动模式、使能、故障与安全信号均未改变。当前速度会话在自身第一条伺服指令被接受后允许此运行标志；`ServoMoveStart` 前仍即时检查机器人静止、程序已停止。暂停/拖动、自动模式、下使能、故障及安全信号变化仍会停止运动。摇杆页仅在自身基坐标 XYZ 会话有效时接受后台验证过的运行状态。就绪错误保留原始状态字段及已接受伺服指令数量。

Linux/Python/XML-RPC is not a hard real-time transport. The 8 ms cycle is the requested cadence, not a timing guarantee; a slow controller/host can stop with a timing diagnostic. The prototype does not replace controller safety limits or a physical stop. Real FR3 servo behavior, timing and stopping distance still need measured commissioning; no physical servo motion was sent while implementing this change.

Linux/Python/XML-RPC 不是硬实时通信。8 ms 是目标周期，不是时序保证；控制器或主机较慢时可能因时序检查而停止。本原型不能替代控制器安全限位或物理停止装置。FR3 实机伺服响应、时序和停止距离仍需现场实测，本次实现过程中未发送实机伺服运动。

For **Calibrated plane XYZ**, first apply/verify the TCP, select a valid plane, use base Wobj0, lock orientation, verify the plane limits and enable Cartesian motion in the XYZ tab. Then choose plane mode and enable joystick teaching. This uses the existing verified `MoveL` path with steps of at most 1 mm, subject to the plane travel box and locked orientation. Total requested travel per held stroke is capped by **Linear travel limit**. Speed is also capped by the XYZ speed setting. Release completes the current step and prevents further steps; it is less continuous than Base XYZ velocity control. Input/focus loss during a pending plane move takes the app's existing stop/disconnect path.

使用 **已标定平面 XYZ** 时，先验证 TCP、选择有效平面、使用基坐标工件 0、锁定姿态、确认行程并启用 XYZ 页的笛卡尔运动，再选择摇杆平面模式并启用。平面模式沿用已有 `MoveL` 路径，每步最多 1 mm，等待反馈确认到位，遵守平面限位并保持锁定姿态。一次持续偏转的累计指令行程不超过 **直线行程上限**，速度还受 XYZ 速度设置约束。松开后完成当前步并停止后续步进，因此连续性弱于基坐标 XYZ 速度运动。运动中失焦或输入丢失会沿用原有停止并断开连接流程。

The capture button works only with the trigger released, all mapped axes centered, and the robot stationary. On **Tool calibration**, it adds a TCP contact. On **Workpiece planes**, it captures the next O/X/Y point. It never calculates/applies a TCP or uploads a plane. Focus loss, unplugging, mode changes, or a polling gap greater than 150 ms disable joystick teaching. Fresh neutral/released input and explicit re-enabling are needed after suspension. A late stop still depends on GUI scheduling and robot communications; the controller travel cap limits each native jog, and this USB input is not a safety-rated enabling switch.

采集按钮仅在扳机松开、映射轴回中且机器人静止时生效：工具标定页添加 TCP 接触点，工件平面页采集下一个 O/X/Y 点。不会自动计算或应用 TCP，也不会上传平面。失焦、拔线、模式切换或超过 150 ms 的轮询间隔会禁用摇杆示教；恢复需松手回中并重新明确启用。停止仍依赖 GUI 调度和机器人通信，原生点动由控制器行程上限约束；USB 输入不是安全级使能开关。

## Profiles and standard Linux tools / 配置与标准工具

Settings are stored independently of calibration in `joystick-profile.json` beside the normal session file. Mapping, inversion, centers, dead zone, response curve and speed persist; an enabled pendant never persists. There are no new Python dependencies. The implementation uses Linux evdev read-only ioctls and does not inject keyboard/mouse events or grab the device.

配置独立于机器人标定，保存在会话文件旁的 `joystick-profile.json` 中。映射、反向、中心、死区、曲线和速度会保存，启用状态不会保存。无需新增 Python 依赖；通过 Linux evdev 只读查询当前状态，不注入键盘/鼠标事件，也不独占设备。

The selected motion mode is also saved when changed and restored at startup, with teaching still disabled. Closed pendant selectors and number fields ignore mouse-wheel changes, allowing the panel to scroll without silently changing axis/button mappings, mode or speed. Open a selector and click a choice, type a number or use its arrow buttons to edit deliberately. The active axis codes, deadman and STOP assignments appear next to the enable control. If a different button is held while deflecting, the readiness message and log identify the configured deadman instead of silently ignoring the attempt. On the verified PXN, the stick/twist axes are **0/1/4**, trigger **288**, STOP **289**, and capture **290**; axis **17** is the hat switch rather than stick X.

切换运动模式时会保存所选模式，下次启动恢复，但示教仍保持禁用。摇杆页中关闭的下拉选择器及数值框忽略鼠标滚轮修改，滚动面板时不会无意更改轴/按钮映射、模式或速度。请打开下拉框并点击选项，或输入数值/点击数值箭头进行明确修改。启用控件旁显示当前轴代码、使能键和 STOP 键；按错使能按钮并偏转时，就绪提示与日志会指出配置的使能键，不再无声忽略。已验证 PXN 的摇杆/扭转轴为 **0/1/4**，扳机 **288**、STOP **289**、采集 **290**；轴 **17** 属于方向帽，不是摇杆 X。

Standard Ubuntu tools were unpacked locally into `.tools/linux-joystick/`, since system installation required a sudo password. Launch them from the project:

```bash
tools/joystick-tools jstest-gtk
tools/joystick-tools jstest /dev/input/by-id/usb-11ff_PC_Game_Controller-joystick
tools/joystick-tools evtest /dev/input/by-id/usb-11ff_PC_Game_Controller-event-joystick
.venv/bin/python tools/check_joystick.py
```

The panel's **Open standard jstest-gtk tool** button also launches the local GUI and suspends joystick teaching. `jstest-gtk` supports joystick axis/button mapping and calibration through the legacy **joydev** interface. Its changes do **not** automatically alter this app's **evdev** input: use the app's mapping/center controls for robot operation. `evtest` is useful for identifying raw axis/button codes. Avoid opening LinuxCNC or another robot-control app against the same joystick during teaching.

系统安装需要 sudo 密码，因此标准 Ubuntu 工具已解包到项目的 `.tools/linux-joystick/`，可用上述命令运行。面板中 **打开标准 jstest-gtk 工具** 按钮也可启动其界面，同时暂停摇杆示教。`jstest-gtk` 使用旧 **joydev** 接口进行轴/按钮映射和校准，其设置不会自动影响本程序的 **evdev** 输入；机器人操作请使用本程序的映射和中心控件。`evtest` 可检查原始代码。示教期间请避免让 LinuxCNC 或其他机器人程序同时使用此摇杆。

To reproduce the local tools on Ubuntu:

```bash
.venv/bin/python tools/install_joystick_tools.py
```

Alternatively install system packages using your own sudo session: `sudo apt install jstest-gtk joystick evtest`. Do not run the robot GUI as root. If opening the input monitor reports permission denied, use the desktop user's normal joystick ACL; do not grant world-writable input access. No global udev or permission changes were made here.

也可自行运行 `sudo apt install jstest-gtk joystick evtest` 安装系统版本。不要以 root 运行机器人界面。若打开设备提示权限不足，请检查桌面用户正常的输入设备 ACL；不要向所有用户开放输入设备写权限。本次未修改全局 udev 或设备权限。

## Verification / 验证

Historical read-only device observations found axis 4 traversed 0–255 during the twist test and trigger presses produced button 288. Local recordings and host profiles are excluded from this repository; retained regression tests use synthetic inputs. The physical device has been opened read-only with the actual input reader; axes, hat and buttons were verified at the evdev ioctl boundary. Recorded/synthetic tests cover center offsets, asymmetric normalization, noise rejection, hysteresis, input held at enable, release, reversal, dominant-axis latching, mode changes, unplugging, GUI polling delays, queued-start cancellation, calibration capture, plane limits, and native jog through the real SDK-worker process in simulation. English and Simplified Chinese UI are covered. No physical robot motion was sent for these tests. Actual direction, stopping distance, physical button assignments and responsiveness remain commissioning checks.

已通过实际输入读取器只读打开实物，并核对轴、方向帽和按钮能力。自动化测试覆盖中心偏移、非对称归一化、噪声与启动余量、启用时已按住输入、松开、反向、主轴锁定、模式切换、拔线、GUI 轮询延迟、取消排队启动、标定采集、平面限位，以及通过真实后台进程运行的仿真点动，并验证中英文界面。这些测试未发送实机运动；实际方向、停止距离、实体按钮对应和响应手感仍需现场确认。

## When buttons jog but the joystick does not / 界面点动正常但摇杆不动

Selecting a different joystick motion mode disables **Enable joystick teaching**. After selecting **Base X / Y / Z**, click the large enable button again, release all buttons and center all three mapped axes (including twist), and wait for **Joystick ready**. Then hold 288 and deflect. The readiness line identifies the exact blocker, such as teaching disabled, missing teaching permission, focus, old feedback, zero throttle, or an off-center axis. A held trigger with a blocked deflection also records the reason in the main log. A rejected or travel-limited stroke requires recentering before retrying. While an accepted velocity stroke is active, moving the throttle up from zero resumes that stroke within its remaining travel budget.

切换摇杆运动模式会取消 **启用摇杆示教**。选择 **基坐标 X / Y / Z** 后，请再次点击大型启用按钮，松开全部按钮并将三个映射轴（含扭转轴）回中，等待 **摇杆已就绪** 后，再按住 288 并偏转。就绪提示会显示具体阻碍：未启用摇杆/示教、焦点、反馈过期、油门为零或某个轴未回中。阻碍期间按住扳机并偏转，也会将原因记录到主日志中。启动被拒或达到行程上限后，需回中重试；已接受的速度操作中，油门从零调高可在剩余行程范围内继续该次操作。

The enable control is now a large amber **Enable joystick teaching** button next to the mode selector, replacing the small checkbox. It turns teal and reads **Disable joystick teaching** when enabled; the heading distinguishes disabled, blocked and ready states.

启用控件已由小复选框改为模式选择器旁的大型琥珀色 **启用摇杆示教** 按钮。启用后按钮变为蓝绿色并显示 **禁用摇杆示教**，标题明确区分未启用、待操作和已就绪状态。

Each enable and native jog now logs the selected motion mode. `Joystick stroke: … speed …%` identifies a native joint/rotation jog; Base XYZ velocity uses the separate servo stream. Normal release/recentering ends a native jog while keeping the connection open. A full shutdown logs `Stop requested: …` before `Motion disabled …`, identifying its source (joystick STOP, Esc, on-screen STOP, disconnect, controller fault or another explicit shutdown). Joystick STOP records the configured button code and all raw pressed button codes; the current PXN mapping uses **289**, separately from trigger **288**. The later “Click Enable joystick teaching” lines describe the disabled state after the stop, not its cause.

每次启用及原生点动操作都会记录所选运动模式。`摇杆操作：…速度…%` 表示原生关节/旋转点动，基坐标 XYZ 速度模式使用独立伺服流。正常松开/回中结束原生点动并保持连接。完整停止会在“运动已禁用……”之前记录“收到停止请求：……”，指出来源（摇杆 STOP、Esc、界面 STOP、断开连接、控制器故障或其他明确停止）。摇杆 STOP 记录已配置的按键代码及当时全部原始按键；当前 PXN 配置使用 **289**，与扳机 **288** 分开。随后“请点击启用摇杆示教”的消息描述停止后的禁用状态，并非停止原因。


## Following-error stop / 跟随误差停止

`Servo TCP following error … exceeds 1 mm` is the application's tracking check, rather than evidence of a lost network connection. It stops motion and invalidates the connection when measured TCP XYZ differs from the last accepted target by more than 1 mm. This is independent of the shared stroke-travel cap and TCP calibration residual.

The stream now uses the pinned SDK's documented **absolute base TCP mode 0**. Previously it sent relative increments but checked their accumulated sum, which assumes the controller accumulates every increment exactly despite tracking lag. The target now sent to the controller is the same absolute target used by the check. The 1 mm threshold, feedback-age limit and timing guards remain unchanged. An actual stall still trips the guard. The error includes actual/target/delta XYZ, accepted-write count, packet age, last RPC duration, speed and stroke travel so further hardware tracking issues can be distinguished.

`伺服 TCP 跟随误差……超过 1 mm` 是应用的跟随检查，不能据此认定网络断开。实测 TCP XYZ 与最后接受的目标相差超过 1 mm 时，程序停止运动并使当前连接失效。此检查独立于共用操作行程上限和 TCP 标定残差。

速度流现在使用固定版本 SDK 所记录的**基坐标绝对 TCP 模式 0**。之前发送相对增量，却检查增量累计值，隐含假设控制器在存在跟随滞后时仍精确累计每次增量；现在发送给控制器的绝对目标与检查使用的目标完全一致。1 mm 阈值、反馈包龄限制及所有时序保护保持不变，实际停滞仍会触发保护。错误日志包含实际/目标/差值 XYZ、已接受指令数、反馈包龄、上次 RPC 耗时、速度及本次行程，便于区分后续实机跟随问题。

The user's higher-speed trace showed 1.064–1.103 mm lag at commanded speeds of 13.64–15.06 mm/s, with feedback 6.9–8.1 ms old and RPC calls around 3 ms. Absolute targets alone did not prevent the trajectory from outrunning the robot. The worker now reduces trajectory speed as measured lag rises from **0.2 to 0.6 mm**, reaching zero advancement at 0.6 mm. These are conservative prototype settings, not measured controller limits. The joystick/throttle request remains a ceiling. Only the reduced velocity is integrated; unserved distance is discarded, so it cannot produce a catch-up burst. Total commanded-path travel remains capped.

The **1 mm hard tracking stop remains independent and unchanged**. Because bounded target lead can hide a stalled plant from that guard, a separate progress check stops when requested speed exceeds 0.2 mm/s, tracking lag exceeds 0.2 mm, and measured displacement remains below 0.05 mm for 0.75 seconds. Neutral/zero throttle resets that progress timer. Input expiry, stale feedback, tool/frame, controller fault and timing checks remain active. No automatic restart or reconnect was added. The live display reports tracking lag and the percentage of stick demand admitted by the governor; the velocity row shows the resulting command.

Tests exercise 35 mm three-axis strokes at 15 and 25 mm/s requests with simulated 80 ms feedback of executed targets, as well as reversals, centering, throttle reduction, fresh-feedback stalls and the independent 1 mm hard stop. These reproduce controller delay in a test plant; actual FR3 behavior still needs verification.

用户提高速度后的实机日志显示：指令速度 13.64–15.06 mm/s 时，滞后达到 1.064–1.103 mm，而反馈包龄仅 6.9–8.1 ms，RPC 耗时约 3 ms。仅改用绝对目标仍不足以避免轨迹超前于机器人。后台现在在实测滞后从 **0.2 mm 增至 0.6 mm** 时逐步降低轨迹速度，达到 0.6 mm 时暂停推进目标。这是保守的原型设置，并非实测控制器性能上限。摇杆与油门请求始终是速度上限。仅对降低后的速度积分，不累积未执行的距离，因此不会突然补追；总指令路径行程上限保持有效。

**独立的 1 mm 跟随误差停止阈值保持不变。** 限制目标超前距离后，机器人完全停滞可能不再触发该阈值，因此新增进度检查：当请求速度大于 0.2 mm/s、跟随滞后大于 0.2 mm，并且 0.75 秒内实测位移不足 0.05 mm 时停止。回中或油门为零会重置进度计时。输入过期、反馈过期、工具/坐标系变化、控制器故障及周期超时检查均保持有效，不会自动重启或重连。界面实时显示跟随滞后及允许执行的摇杆请求比例，速度行显示最终指令。百分比速度控件现标为“关节 / 旋转 / 平面速度上限”，不影响基坐标 XYZ 的 mm/s 设置。

测试覆盖模拟 80 ms 延迟下，以 15 和 25 mm/s 请求完成 35 mm 三轴操作，以及反向、回中、降低油门、反馈正常但机器人停滞、独立 1 mm 停止保护。这些是模拟控制器测试，FR3 实机效果仍需验证。
