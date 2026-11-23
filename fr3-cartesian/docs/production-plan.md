# From prototype to production / 从原型到产线软件

## Current foundation / 当前基础

The application has an isolated robot process, calibration geometry, single-flight motion requests, bounded error handling, a simulator, and an English/Simplified Chinese UI. Operator errors carry an English template and named values across the process boundary; the GUI translates them at display time. Robot coordinates, units, SDK calls and calibration files are independent of display language. The translation catalog ships with the Python package.

当前软件已具备独立机器人进程、标定几何计算、单条在途运动指令、超时与错误处理、仿真器，以及中英文界面。操作员错误消息以英文模板和命名参数跨进程传递，由界面显示时翻译。坐标、单位、SDK 调用及标定文件不依赖显示语言。翻译目录随 Python 软件包发布。

This remains a commissioning prototype. Production development should proceed against the actual controller, gripper, fixtures and line interfaces, with explicit acceptance criteria for each milestone.

当前版本仍是调试原型。后续开发应结合实际控制器、夹爪、夹具和产线接口开展，并为每个阶段明确验收标准。

The operator UI now focuses on calibration, native joint/base jogs, calibrated XYZ positioning, and a basic static array-to-fixed-destination coordinate sequence. The joystick pendant was removed after hardware trials did not meet the expected responsiveness. Its backend and tests remain as historical experiments; they are not the foundation for the next pick/place cycle.

当前界面聚焦标定、原生关节 / 基坐标点动和标定平面 XYZ 定位。摇杆实机试用未达到预期手感，已从界面移除；相关后台与测试仅作为历史实验保留。下一步应围绕取放任务运动迭代。

## Proposed milestones / 建议迭代阶段

| Stage / 阶段 | Deliverable / 交付内容 | Acceptance / 验收条件 |
| --- | --- | --- |
| 1. Controller integration / 控制器适配 | Confirm SDK for V3.8.25-QX; record actual robot identity, firmware and connection settings. / 确认匹配 QX 固件的 SDK，记录机器人身份、固件和连接参数。 | Verify feedback frames, TCP conventions, tool selection, bounded moves, stop delivery and recovery on the actual FR3. / 实机验证反馈、TCP 约定、工具选择、小步运动、停止与恢复。 |
| 2. Calibration records / 标定档案 | Versioned tool/fixture records with robot serial, tool identity, residuals, timestamps and verification points. / 标定记录包含机器人序列号、工具身份、残差、时间与验证点。 | Reject mismatched or obsolete records; meet measured positioning tolerance over the intended workspace. / 拒绝不匹配或过期记录，工作区内的实测定位精度达标。 |
| 3. Pick/place recipe / 取放配方 | Explicit cycle state machine: approach, descend, grip, verify, retract, transfer, release and verify. Configurable gripper I/O, timeouts and sensor feedback. / 明确接近、下降、夹取、确认、抬升、转移、释放与确认状态；配置夹爪 I/O、超时和反馈。 | Pass dry runs and supervised material trials; no automatic continuation after an uncertain grip, failed move or restart. / 通过空跑与受控带料试验；夹取不确定、运动失败或重启后不自动续跑。 |
| 4. Operator workflow / 操作员流程 | Separate commissioning and operator screens; setup permissions, recipe selection, bilingual alarms and guided recovery. / 分离调试与操作界面，增加设置权限、配方选择、双语报警与恢复指引。 | Operators can run and recover agreed scenarios in either language; setup changes are recorded. / 操作员可用任一语言完成规定操作与恢复，设置变更可追踪。 |
| 5. Line integration / 产线集成 | PLC ready/busy/done/fault handshake, part presence, recipe identifiers and traceable cycle outcomes. / PLC 就绪、忙碌、完成与故障握手，工件检测、配方标识和循环结果追溯。 | Verify handshake timeouts, duplicate requests, communications loss, power loss and restart behavior with the line controller. / 联机验证握手超时、重复请求、断网、断电与重启行为。 |
| 6. Release and soak / 发布与持续运行 | Reproducible installer, persistent structured event/alarm history, configuration backups and tested rollback. / 可复现安装包、持久化结构化事件与报警、配置备份和回退。 | Complete agreed-duration soak tests, cycle-time targets and site acceptance with the installed cell. / 完成约定时长的连续运行、节拍验证与现场验收。 |

## Implementation direction / 实现方向

Keep the GUI as a presentation layer. Move the application workflow into a controller/state-machine layer before adding automatic cycles; the worker should retain exclusive SDK ownership. Model recipes, machine configuration and calibration as distinct versioned records. Use stable event IDs and structured parameters for future persistent logs; the current translated on-screen log is bounded in memory and is not an audit database.

保持 GUI 作为显示与交互层。在增加自动循环前，将业务流程提取至控制器 / 状态机层；机器人进程继续独占 SDK。配方、设备配置与标定应分别建模并管理版本。持久化日志应使用稳定事件编号与结构化参数；当前界面日志只在内存中保留有限条目，并不是审计数据库。

The static pick/place prototype emits ordered straight TCP moves but has no gripper or vision integration. The next engineering milestone is measured controller/SDK and calibration verification before adding task-specific approach, grip verification and recovery. Gripper type, available I/O and sensors, PLC protocol, positioning tolerance and target cycle time are inputs to the subsequent cycle design. The software stop and TCP travel box do not replace the cell's physical safety system or assessment.

下一项工程里程碑是控制器 / SDK 适配和标定精度实测。夹爪类型、可用 I/O 与传感器、PLC 协议、定位公差及目标节拍，是后续自动循环设计的输入。软件停止和 TCP 行程限制不能代替工作站的物理安全系统及其评估。
