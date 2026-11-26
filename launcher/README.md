# Pick & Place Workbench / 取放工作台启动器

Use the **Pick & Place Workbench / 取放工作台启动器** desktop shortcut or find it
in the application menu. Select a tool, review its configuration and click
**Start / 启动**. After editing Python source, click **Restart / 重启** to reload it.
There is no build/copy step. It runs the sources in this repository.
If GNOME asks to trust the desktop shortcut, right-click it and choose **Allow
Launching** once; the application-menu entry works independently of this flag.

## Included tools

- **Camera preview / 相机预览**: select MindVision/Hikvision, click **Find cameras /
  查找相机** and choose a serial number, or type the serial directly. The selected
  camera can open and start live view automatically. A missing serial never opens
  another camera. With a blank serial only a unique detected device is eligible
  for automatic opening. Multiple detected devices require manual selection.
  MindVision defaults to the installed library that works with GTK_demo.
- **FR3 / Cartesian Studio**: robot IPv4 address, real robot/simulator backend,
  language, vendor SDK folder, calibration session and interpreter. It always
  starts **disconnected**. Connect and enable motion inside Studio as usual.

Settings are per tool. **Save / 保存参数** persists them for the next launch;
Start/Restart also saves first. Settings do not change an already-running tool.
Resolution/exposure/gamma/contrast remain inside the camera preview. Calibration
and motion settings remain inside Studio.

**Network / 网络** shows local adapter addresses and opens the configured robot
WebUI. The configured robot address selects the endpoint used by Studio; the
launcher does not change the robot/camera IP or operating-system adapters.

## Lifetimes, restart and logs

Each tool runs in its own process with its own Python environment. A detached
supervisor captures stdout/stderr, keeps rotating log files, and holds a kernel
instance lock for the tool's full lifetime. Closing/reopening the launcher leaves
those tools running and restores their status/logs. Duplicate starts and aliases
sharing an `instance_key` are blocked within this launcher installation.

**Stop / 正常退出** asks the application to close normally. **Restart / 重启** first
requests this close, then launches current source only after the previous process
has exited. The built-in apps handle SIGTERM on the Qt event loop and run their
existing close cleanup (FR3 stop channel/SDK worker cleanup or camera cleanup).
The launcher does not forcibly kill a robot studio or automatically restart a
crashed application. An unresponsive process remains stopping until it exits;
use the application/controller's normal stop procedure if intervention is needed.
Closing the launcher cancels any queued relaunch; an already-requested normal
close can still finish. Closing the launcher sends no additional stop commands.

An already-running tool started from a terminal is outside this supervisor's
lifetime management. Built-in entries detect these module processes, show
**External / 外部运行**, and block a duplicate start until you close the old
window normally. It never kills or adopts that window. Use one studio per physical robot. Generic tools
must remain foreground processes and implement SIGTERM/normal shutdown to support
Stop/Restart; commands are never run through a shell.

**Live logs / 运行日志** shows recent output for the selected tool, including
camera status/errors and Studio operator messages. Full files are in:

```text
~/.local/state/pickplace-launcher/
```

Use **Log folder / 日志目录** to open it. Each log retains 2 MiB plus three rotated
backups. Launch records include the source directory, argv, timestamps, PID and
exit code. Tool stdout is unbuffered. No robot commands are sent by the launcher.

## Installation and source locations

Installed on this workstation already. To install again or on another host:

```bash
cd /home/hotcat/Downloads/fr3robot/PickAndPlaceTilted
./tools/install-workbench.sh
```

This creates the camera/launcher environment if missing, seeds user configuration
only when missing, and installs menu/desktop shortcuts. It does not start tools
or replace saved configuration/calibration. It uses `/usr/bin/python3` for a new
environment, avoiding Conda's system Qt/plugin conflicts. Existing environments
are reused. The robot environment and vendor SDK must be installed separately;
set their paths in the launcher on another workstation.

To run directly or validate configuration:

```bash
./launch-workbench.sh
./launch-workbench.sh --check-config
./launch-workbench.sh --config /path/to/site.json
```

At initial configuration on this host, FR3 reuses:

```text
Interpreter: /home/hotcat/Downloads/fr3robot/.venv/bin/python
SDK:         /home/hotcat/Downloads/fr3robot/vendor
Calibration: /home/hotcat/Downloads/fr3robot/calibration-session.json
```

The actual source imported by this entry is:

```text
/home/hotcat/Downloads/fr3robot/PickAndPlaceTilted/fr3-cartesian/fr3_cartesian/
```

The launcher passes this repository's source directory first in `PYTHONPATH` and
an explicit session path, preventing an old editable install or different working
directory from silently loading the wrong source/calibration. **Edit this source
when using this launcher**, rather than the older top-level `fr3_cartesian` copy.
Opening the launcher itself never opens a robot connection or camera stream.

## Extending the tool catalogue

The per-user catalogue is:

```text
~/.config/pickplace-launcher/config.json
```

Use **Edit catalogue / 编辑目录** to add/edit JSON inside the launcher, or edit
it externally and click **Reload / 重新读取**. Invalid files are preserved and
reported. A digest check refuses to overwrite external edits until reloaded.
`XDG_CONFIG_HOME` and `XDG_STATE_HOME` override the configuration/state roots.
Defaults live in [defaults.json](defaults.json), and do not overwrite deployed
configuration on update.

A new foreground tool needs only another entry in `tools`:

```json
{
  "id": "inspection-tool",
  "instance_key": "inspection-tool",
  "name": "Inspection / 检测工具",
  "description": "Example future inspection workbench",
  "command": ["{python}", "-u", "-m", "inspection", "--controller", "{controller_ip}"],
  "cwd": "{repo}",
  "python_paths": ["{repo}"],
  "parameters": [
    {"key": "python", "label": "Python", "type": "path", "default": "/path/to/venv/bin/python"},
    {"key": "controller_ip", "label": "Controller IP", "type": "ipv4", "default": "192.168.1.10"},
    {"key": "debug", "label": "Debug", "type": "boolean", "default": false, "env": "INSPECT_DEBUG"}
  ],
  "values": {},
  "environment": {}
}
```

The UI builds editors from `parameters`. Supported types: `string`, `path`,
`ipv4`, `choice` (`choices` list), `boolean`, `integer` (`min`/`max`). `required`
may be false for optional text; `help` supplies a tooltip. A path can specify
`path_kind: "directory"` for a directory chooser. `{repo}` and parameter keys
substitute individual argv/path/environment tokens; values containing spaces
remain a single argument and are never shell-interpreted. Boolean environment
bindings use `1`/`0`. Each tool must implement the flags/environment it is given.
An optional `exclusive_module` such as `fr3_cartesian` also detects existing
same-user Python `-m` processes started outside the launcher and blocks duplicates.

Optional `device_discovery` accepts a `command` argument array and the target
`parameter` (a string with `device_selector: true`). Its foreground helper must
print `CAMERA_DEVICES_JSON:` followed by a JSON list of `{ "name", "sn" }`.
Discovery runs separately without opening a camera, times out after 15 seconds,
and populates the selector. This extension point can serve future SDK backends.

## 简体中文操作说明

1. 双击桌面的 **取放工作台启动器 / Pick & Place Workbench**，选择相机预览或 FR3。
2. 相机点击 **查找相机**，选择序列号；FR3 设置机器人 IP、SDK、标定文件与语言。
3. 点击 **启动**，在工具窗口操作。FR3 始终先处于断开连接状态。
4. 修改仓库内源码后点击 **重启**，启动器等待旧程序正常退出，再加载新源码。
5. 日志页查看实时输出；关闭启动器不关闭正在运行的工具，重新打开仍可管理。
6. 扩展工具时点击 **编辑目录**，在 JSON 的 `tools` 数组中添加命令和参数定义。
   不需要为每个新工具修改启动器代码。外部编辑 JSON 后点击 **重新读取**。

已通过终端独立启动的旧窗口不受启动器管理；切换前先关闭旧相机/FR3 窗口。
机器人端点 IP 在此配置，系统网卡/相机硬件 IP 不会被启动器修改。

## Architecture and verification

- `config.py`: schema, validation, safe argv expansion, atomic persistence.
- `processes.py`: Linux instance locks, detached supervisor, run-scoped close
  requests, rotating log capture, exit records.
- `ui.py`: catalogue-driven forms, lifecycle controls, logs, read-only networking.
- `runtime.py`: cooperative Qt close handling used by built-in apps.
- Existing tools retain their own hardware SDKs and separate processes/environments.

```bash
QT_QPA_PLATFORM=offscreen .venv-camera/bin/python -m pytest tests/launcher tests/test_camera_window.py -q
```

Tests exercise persistent supervisor ownership, UTF-8 live output before exit,
duplicate lock protection, restart ordering, exit diagnostics, parameter mapping,
configuration conflict handling and exact-serial camera auto-preview behavior.
