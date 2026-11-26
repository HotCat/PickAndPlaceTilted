# PickAndPlaceTilted

Vision and robot commissioning tools for a pick-and-place project. The camera
workbenches and robot application currently run independently; camera registration
does not send robot motion.

| Component | Entry point | Guide |
| --- | --- | --- |
| MindVision / Hikvision aiming preview / 工业相机瞄准预览 | `./preview-camera.sh` (MindVision) or `./preview-camera.sh --backend hikvision` | [Launch, SDK setup and aiming notes / 启动说明](docs/camera-preview.md) |
| Tilted-camera rectification / 倾斜相机校正 | `python app.py` | Camera calibration and settings are available in the application. |
| PCB CAD registration / PCB CAD 配准 | `python tools/select_pcb_holes.py` | [Registration, recipes and Hikvision capture](docs/pcb_cad_localization.md) |
| FAIRINO FR3 calibration and XYZ positioning / FR3 标定与 XYZ 定位 | From `fr3-cartesian/`: `.venv/bin/python -m fr3_cartesian` | [English](fr3-cartesian/README.md) · [简体中文](fr3-cartesian/README.zh-CN.md) |

The FR3 application has its own environment, dependency pins, SDK installer and
tests under **[fr3-cartesian/](fr3-cartesian/)**. It provides TCP pivot calibration,
multiple workpiece planes, native joint/base jogs, independent TCP jog speed,
calibration persistence, and English/Simplified Chinese UI switching. It starts
disconnected and includes a simulator. Joystick teaching is retired from its UI.

FR3 程序使用独立 Python 环境，支持 TCP 标定、多个工件平面、原生关节点动与基坐标
点动、独立 TCP 点动速度、标定保存和中英文切换。启动时不连接机器人，可先使用仿真器。
当前尚未实现相机到机器人自动联动、夹爪控制或自动取放循环。

See the [FR3 architecture notes](fr3-cartesian/docs/architecture.md) for coordinate
conventions, SDK compatibility, test commands and the intended vision integration
boundary, and the [production plan](fr3-cartesian/docs/production-plan.md) for
remaining recipe, gripper and line integration work.
