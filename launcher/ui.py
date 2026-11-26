"""A JSON-driven inspector; adding another foreground tool needs no UI code."""
from __future__ import annotations

import copy
import json
from pathlib import Path
import time

from PySide6.QtCore import QProcess, QProcessEnvironment, Qt, QTimer, QUrl
from PySide6.QtGui import QDesktopServices, QFontDatabase
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QFileDialog, QFormLayout, QHBoxLayout, QLabel,
    QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QScrollArea,
    QSpinBox, QSplitter, QTabWidget, QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget,
)

from .config import check_spec, launch_spec, parameter_value
from .processes import ProcessManager, external_instances


class LauncherWindow(QMainWindow):
    def __init__(self, store, manager=None):
        super().__init__()
        self.store = store
        self.config = copy.deepcopy(store.data)
        self.manager = manager or ProcessManager()
        self.selected_id = None
        self.editors = {}
        self.rows = {}
        self.pending_restarts = {}
        self.discovery_process = None
        self.discovered = {}
        self.external_cache = {}
        self.external_checked = 0.
        self.setWindowTitle('Pick & Place Workbench / 取放工作台启动器')
        self.resize(1220, 840)
        self.setMinimumSize(900, 620)
        self.setStyleSheet('''
          QWidget { background: #202832; color: #e7edf5; }
          QLabel#heading { font-size: 24px; font-weight: 600; }
          QLineEdit, QComboBox, QSpinBox, QPlainTextEdit, QTreeWidget {
            background: #141c25; color: #e7edf5; border: 1px solid #405266;
            selection-background-color: #265f89; selection-color: white; padding: 5px;
          }
          QComboBox QAbstractItemView { background: #141c25; color: #e7edf5; }
          QHeaderView::section, QTabBar::tab { background: #2b3948; color: #e7edf5; padding: 9px; }
          QTabBar::tab:selected { background: #265f89; }
          QPushButton { background: #34485b; color: #ffffff; padding: 8px 12px;
                        border: 1px solid #51677c; border-radius: 4px; }
          QPushButton:hover { background: #42627c; }
          QPushButton#start { background: #167052; }
          QPushButton:disabled { background: #27333f; color: #8d9aa8; }
          QCheckBox { padding: 6px; }
          QTreeWidget::item { padding: 10px 4px; }
          QTreeWidget::item:selected { background: #265f89; color: white; }
          QScrollArea { border: none; }
        ''')
        page = QWidget()
        self.setCentralWidget(page)
        layout = QVBoxLayout(page)
        layout.setContentsMargins(20, 16, 20, 16)
        layout.addWidget(QLabel('Pick & Place Workbench / 取放工作台', objectName='heading'))
        layout.addWidget(QLabel('Choose a tool → configure → Start. After source edits, use Restart. / 选择工具、配置后启动；修改源码后点击重启。'))
        top = QHBoxLayout()
        path = QLabel(str(store.path))
        path.setTextInteractionFlags(Qt.TextSelectableByMouse)
        top.addWidget(path, 1)
        self.edit_config_button = QPushButton('Edit catalogue / 编辑目录')
        self.edit_config_button.clicked.connect(self.edit_catalogue)
        top.addWidget(self.edit_config_button)
        self.reload_button = QPushButton('Reload / 重新读取')
        self.reload_button.clicked.connect(self.reload_config)
        top.addWidget(self.reload_button)
        layout.addLayout(top)
        self.notice = QLabel('')
        self.notice.setWordWrap(True)
        self.notice.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.notice)
        split = QSplitter()
        layout.addWidget(split, 1)
        self.tool_list = QTreeWidget()
        self.tool_list.setRootIsDecorated(False)
        self.tool_list.setHeaderLabels(['Tool / 工具', 'State / 状态'])
        self.tool_list.setColumnWidth(0, 245)
        self.tool_list.currentItemChanged.connect(self.select_tool)
        split.addWidget(self.tool_list)
        right = QWidget()
        content = QVBoxLayout(right)
        self.title = QLabel('', objectName='heading')
        self.description = QLabel('')
        self.description.setWordWrap(True)
        self.status = QLabel('')
        self.status.setWordWrap(True)
        self.status.setTextInteractionFlags(Qt.TextSelectableByMouse)
        content.addWidget(self.title)
        content.addWidget(self.description)
        content.addWidget(self.status)
        self.tabs = QTabWidget()
        self.parameter_scroll = QScrollArea()
        self.parameter_scroll.setWidgetResizable(True)
        self.tabs.addTab(self.parameter_scroll, 'Configuration / 参数')
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(QFontDatabase.systemFont(QFontDatabase.FixedFont))
        self.log_view.setMaximumBlockCount(3000)
        self.tabs.addTab(self.log_view, 'Live logs / 运行日志')
        network = QWidget()
        net_layout = QVBoxLayout(network)
        net_layout.addWidget(QLabel('Local adapter addresses (read only) / 本机网卡地址（只读）'))
        net_row = QHBoxLayout()
        self.network_button = QPushButton('Refresh adapters / 刷新网卡')
        self.network_button.clicked.connect(self.refresh_network)
        net_row.addWidget(self.network_button)
        web_button = QPushButton('Robot WebUI / 机器人网页')
        web_button.clicked.connect(self.open_robot_webui)
        net_row.addWidget(web_button)
        net_layout.addLayout(net_row)
        self.network_view = QPlainTextEdit()
        self.network_view.setReadOnly(True)
        net_layout.addWidget(self.network_view)
        note = QLabel('Set robot IP in Configuration. Camera selection uses its SDK and serial number.\n'
                      'Changing these fields does not change the robot/camera device IP or the OS network configuration.\n'
                      '机器人 IP 在参数页设置，相机按 SDK 与序列号选择；此处不修改设备 IP 或系统网卡配置。')
        note.setWordWrap(True)
        net_layout.addWidget(note)
        self.tabs.addTab(network, 'Network / 网络')
        content.addWidget(self.tabs, 1)
        self.command_label = QLabel('')
        self.command_label.setWordWrap(True)
        self.command_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        content.addWidget(self.command_label)
        actions = QHBoxLayout()
        self.save_button = QPushButton('Save / 保存参数')
        self.save_button.clicked.connect(self.save_parameters)
        actions.addWidget(self.save_button)
        self.find_devices_button = QPushButton('Find cameras / 查找相机')
        self.find_devices_button.clicked.connect(self.find_devices)
        self.logs_button = QPushButton('Log folder / 日志目录')
        self.logs_button.clicked.connect(lambda: QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.manager.directory))))
        utilities = QHBoxLayout()
        utilities.addWidget(self.find_devices_button)
        utilities.addWidget(self.logs_button)
        utilities.addStretch()
        content.insertLayout(3, utilities)
        actions.addStretch()
        self.start_button = QPushButton('Start / 启动', objectName='start')
        self.stop_button = QPushButton('Stop / 正常退出')
        self.restart_button = QPushButton('Restart / 重启')
        self.start_button.clicked.connect(self.start_selected)
        self.stop_button.clicked.connect(self.stop_selected)
        self.restart_button.clicked.connect(self.restart_selected)
        for button in (self.start_button, self.stop_button, self.restart_button):
            actions.addWidget(button)
        content.addLayout(actions)
        split.addWidget(right)
        split.setSizes([360, 820])
        footer = QLabel('Closing this launcher leaves tools running. FR3 starts disconnected; Stop/Restart waits for normal shutdown.\n'
                        '关闭启动器不会关闭工具。FR3 启动时不连接机器人；退出或重启会等待程序正常关闭。')
        footer.setWordWrap(True)
        layout.addWidget(footer)
        self.network_process = None
        self.populate()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)
        self.refresh()

    def tool(self):
        return next((t for t in self.config['tools'] if t['id'] == self.selected_id), None)

    def key(self, tool):
        return tool.get('instance_key', tool['id'])

    def populate(self):
        self.selected_id, self.editors = None, {}
        self.tool_list.clear()
        self.rows = {}
        for tool in self.config['tools']:
            row = QTreeWidgetItem([tool['name'], ''])
            row.setData(0, Qt.UserRole, tool['id'])
            self.rows[tool['id']] = row
            self.tool_list.addTopLevelItem(row)
        if self.rows:
            self.tool_list.setCurrentItem(next(iter(self.rows.values())))

    def remember_edits(self):
        tool = self.tool()
        if tool:
            values = tool.setdefault('values', {})
            for key, editor in self.editors.items():
                if isinstance(editor, QCheckBox):
                    values[key] = editor.isChecked()
                elif isinstance(editor, QComboBox):
                    values[key] = editor.currentText()
                elif isinstance(editor, QSpinBox):
                    values[key] = editor.value()
                else:
                    values[key] = editor.text()

    def select_tool(self, current, previous=None):
        self.remember_edits()
        self.selected_id = current.data(0, Qt.UserRole) if current else None
        tool = self.tool()
        self.editors = {}
        if not tool:
            return
        self.title.setText(tool['name'])
        self.description.setText(tool.get('description', ''))
        body = QWidget()
        form = QFormLayout(body)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        for definition in tool.get('parameters', []):
            value = parameter_value(tool, definition)
            kind = definition['type']
            if kind == 'boolean':
                editor = QCheckBox()
                editor.setChecked(value)
            elif kind == 'choice' or definition.get('device_selector'):
                editor = QComboBox()
                editor.setEditable(kind != 'choice')
                editor.addItems(definition.get('choices', []))
                if definition.get('device_selector'):
                    for device in self.discovered.get(tool['id'], []):
                        editor.addItem(device['sn'])
                        editor.setItemData(editor.count()-1, device['name'], Qt.ToolTipRole)
                editor.setCurrentText(value)
            elif kind == 'integer':
                editor = QSpinBox()
                editor.setRange(definition.get('min', -2147483648), definition.get('max', 2147483647))
                editor.setValue(value)
            else:
                editor = QLineEdit(value)
            editor.setToolTip(definition.get('help', ''))
            self.editors[definition['key']] = editor
            if kind == 'path':
                row = QWidget()
                layout = QHBoxLayout(row)
                layout.setContentsMargins(0, 0, 0, 0)
                layout.addWidget(editor)
                button = QPushButton('…')
                button.setMaximumWidth(36)
                button.clicked.connect(lambda _=False, edit=editor, definition=definition: self.browse_path(edit, definition))
                layout.addWidget(button)
                form.addRow(definition['label'], row)
            else:
                form.addRow(definition['label'], editor)
        self.parameter_scroll.setWidget(body)
        self.find_devices_button.setVisible(bool(tool.get('device_discovery')))
        self.log_view.clear()
        self.refresh()

    def browse_path(self, editor, definition):
        # A non-native file chooser follows our palette on dark desktops.
        if definition.get('path_kind') == 'directory':
            selected = QFileDialog.getExistingDirectory(self, 'Select folder / 选择目录', editor.text(),
                                                        QFileDialog.DontUseNativeDialog)
            if selected:
                editor.setText(selected)
            return
        dialog = QFileDialog(self, 'Select path / 选择路径', str(Path(editor.text()).parent))
        dialog.setOption(QFileDialog.DontUseNativeDialog)
        dialog.setFileMode(QFileDialog.AnyFile)
        if dialog.exec():
            editor.setText(dialog.selectedFiles()[0])

    def save_parameters(self):
        try:
            self.remember_edits()
            self.store.save(self.config)
            self.notice.setText('Saved for the next launch / 已保存，下次启动生效')
            return True
        except (ValueError, OSError) as exc:
            self.show_error(exc)
            return False

    def prepared_spec(self):
        if not self.save_parameters():
            return None
        spec = launch_spec(self.tool(), self.store.repository)
        check_spec(spec)
        return spec

    def start_selected(self):
        if not self.tool():
            return
        try:
            spec = self.prepared_spec()
            if spec:
                self.manager.launch(self.key(self.tool()), spec)
                self.notice.setText('Started / 已启动')
                self.tabs.setCurrentWidget(self.log_view)
                self.refresh()
        except (OSError, ValueError) as exc:
            self.show_error(exc)

    def stop_selected(self):
        if self.tool():
            self.pending_restarts.pop(self.key(self.tool()), None)
            self.manager.stop(self.key(self.tool()))
            self.notice.setText('Requested normal close; waiting / 已请求正常退出，正在等待')

    def restart_selected(self):
        if not self.tool():
            return
        try:
            spec = self.prepared_spec()
            if not spec:
                return
            key = self.key(self.tool())
            if self.manager.status(key)['running']:
                self.pending_restarts[key] = spec
                self.manager.stop(key)
                self.notice.setText('Restart queued after normal exit / 等待正常退出后重新启动')
            else:
                self.manager.launch(key, spec)
            self.tabs.setCurrentWidget(self.log_view)
        except (OSError, ValueError) as exc:
            self.show_error(exc)

    def refresh(self):
        if time.monotonic()-self.external_checked > 2:
            self.external_cache = {t['id']: external_instances(t.get('exclusive_module')) for t in self.config['tools']}
            self.external_checked = time.monotonic()
        for key, spec in list(self.pending_restarts.items()):
            if not self.manager.status(key)['running']:
                self.pending_restarts.pop(key)
                try:
                    self.manager.launch(key, spec)
                    self.notice.setText('Restarted with current source / 已用当前源码重新启动')
                except (OSError, ValueError) as exc:
                    self.show_error(exc)
        for tool in self.config['tools']:
            state = self.manager.status(self.key(tool))
            self.rows[tool['id']].setText(1, 'External / 外部运行' if not state['running'] and self.external_cache.get(tool['id']) else state['state'])
        tool = self.tool()
        if not tool:
            for button in (self.start_button, self.stop_button, self.restart_button, self.save_button):
                button.setEnabled(False)
            return
        state = self.manager.status(self.key(tool))
        external = self.external_cache.get(tool['id'], []) if not state['running'] else []
        self.status.setText(f'Existing window outside launcher (PID {external}). Close it normally first. / 请先正常关闭原有工具窗口。' if external else
                            f"{state['state']}  ·  PID {state.get('pid', '—')}  ·  exit {state.get('exit_code', '—')}\n{state.get('message', '')}")
        self.start_button.setEnabled(not state['running'] and not external)
        self.stop_button.setEnabled(state['running'])
        self.restart_button.setEnabled(not external and state['state'] != 'stopping' and self.key(tool) not in self.pending_restarts)
        self.save_button.setEnabled(True)
        log = self.manager.log_tail(self.key(tool))
        if self.log_view.toPlainText() != log:
            bar = self.log_view.verticalScrollBar()
            at_end, previous = bar.value() >= bar.maximum()-2, bar.value()
            self.log_view.setPlainText(log)
            bar.setValue(bar.maximum() if at_end else previous)
        try:
            spec = launch_spec(tool, self.store.repository)
            self.command_label.setText('Source / 源码: ' + spec['cwd'])
        except ValueError as exc:
            self.command_label.setText(str(exc))

    def reload_config(self):
        try:
            self.remember_edits()
            if self.config != self.store.data:
                result = QMessageBox.question(self, 'Reload / 重新读取', 'Discard unsaved edits? / 放弃未保存的修改？')
                if result != QMessageBox.Yes:
                    return
            self.store.load()
            self.config = copy.deepcopy(self.store.data)
            self.populate()
            self.notice.setText('Catalogue reloaded / 程序目录已更新')
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.show_error(exc)

    def edit_catalogue(self):
        # JSON editing stays in the launcher: no association with a particular
        # desktop editor is required on a production workstation.
        from PySide6.QtWidgets import QDialog, QDialogButtonBox
        self.remember_edits()
        dialog = QDialog(self)
        dialog.setWindowTitle('Tool catalogue / 工具目录 JSON')
        dialog.resize(900, 700)
        layout = QVBoxLayout(dialog)
        layout.addWidget(QLabel('Add a tool with command, cwd and typed parameters. Changes affect the next launch.'))
        editor = QPlainTextEdit(json.dumps(self.config, ensure_ascii=False, indent=2))
        layout.addWidget(editor)
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        layout.addWidget(buttons)
        buttons.rejected.connect(dialog.reject)

        def save():
            try:
                data = json.loads(editor.toPlainText())
                self.store.save(data)
                self.config = copy.deepcopy(self.store.data)
                self.populate()
                dialog.accept()
            except (ValueError, OSError, KeyError, TypeError) as exc:
                self.show_error(exc)
        buttons.accepted.connect(save)
        dialog.exec()

    def refresh_network(self):
        if self.network_process and self.network_process.state() != QProcess.NotRunning:
            return
        process = QProcess(self)
        self.network_process = process
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.network_view.insertPlainText(bytes(process.readAllStandardOutput()).decode(errors='replace')))
        process.errorOccurred.connect(lambda _: self.network_view.appendPlainText(process.errorString()))
        self.network_view.clear()
        process.start('ip', ['-brief', 'address'])

    def find_devices(self):
        tool = self.tool()
        if not tool or not tool.get('device_discovery'):
            return
        if self.discovery_process and self.discovery_process.state() != QProcess.NotRunning:
            return
        self.remember_edits()
        try:
            spec = launch_spec(dict(tool, command=tool['device_discovery']['command']), self.store.repository)
            check_spec(spec)
        except (OSError, ValueError) as exc:
            self.show_error(exc)
            return
        process = QProcess(self)
        self.discovery_process = process
        environment = QProcessEnvironment.systemEnvironment()
        for key, value in spec['environment'].items():
            environment.insert(key, value)
        process.setProcessEnvironment(environment)
        process.setWorkingDirectory(spec['cwd'])
        process.setProcessChannelMode(QProcess.MergedChannels)
        self.notice.setText('Finding cameras (not opening them) / 正在查找相机，不打开设备…')

        def finished(code, _status):
            text = bytes(process.readAllStandardOutput()).decode(errors='replace')
            try:
                if code:
                    raise ValueError(text[-3000:] or 'Camera discovery failed')
                line = next((line for line in text.splitlines() if line.startswith('CAMERA_DEVICES_JSON:')), None)
                if line is None:
                    raise ValueError(text[-3000:] or 'No device list returned')
                devices = json.loads(line.split(':', 1)[1])
                if not isinstance(devices, list) or any(not isinstance(d, dict) or not isinstance(d.get('sn'), str) or not isinstance(d.get('name'), str) for d in devices):
                    raise ValueError('Invalid device list')
                self.discovered[tool['id']] = devices
                if self.selected_id == tool['id']:
                    editor = self.editors[tool['device_discovery']['parameter']]
                    current = editor.currentText()
                    editor.clear()
                    for d in devices:
                        editor.addItem(d['sn'])
                        editor.setItemData(editor.count()-1, d['name'], Qt.ToolTipRole)
                    editor.setCurrentText(current or (devices[0]['sn'] if len(devices) == 1 else ''))
                self.notice.setText('Detected / 已找到: ' + (', '.join(d['name'] for d in devices) or '0 cameras'))
            except (OSError, ValueError, TypeError, KeyError) as exc:
                self.show_error(exc)
        process.finished.connect(finished)
        process.errorOccurred.connect(lambda _: self.notice.setText(process.errorString()))
        process.start(spec['command'][0], spec['command'][1:])
        QTimer.singleShot(15000, lambda: process.kill() if process.state() != QProcess.NotRunning else None)

    def open_robot_webui(self):
        tool = next((t for t in self.config['tools'] if t['id'] == 'fr3-studio'), None)
        if tool:
            self.remember_edits()
            definition = next(d for d in tool['parameters'] if d['key'] == 'robot_ip')
            from .config import validate_value
            try:
                ip = validate_value(definition, parameter_value(tool, definition))
                QDesktopServices.openUrl(QUrl(f'http://{ip}/'))
            except ValueError as exc:
                self.show_error(exc)

    def show_error(self, exc):
        self.notice.setText(str(exc))
        QMessageBox.warning(self, 'Launcher / 启动器', str(exc))

    def closeEvent(self, event):
        # Detached supervisors own tool lifetimes. Closing this window never
        # interrupts the robot/camera; an unfinished restart is simply cancelled.
        self.pending_restarts.clear()
        self.timer.stop()
        # Only the read-only helper belongs to this UI, unlike managed tools.
        if self.discovery_process and self.discovery_process.state() != QProcess.NotRunning:
            self.discovery_process.blockSignals(True)
            self.discovery_process.kill()
            self.discovery_process.waitForFinished(300)
        event.accept()
