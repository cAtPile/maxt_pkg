#!/usr/bin/env python3
"""交互式编排顺序任务；可只读订阅 ROS 状态，不执行飞行。"""

import argparse
import curses
import json
import locale
import math
import os
from pathlib import Path
import sys
import threading
import time
import xml.etree.ElementTree as ET
from xml.dom import minidom


# 仅列出 providedPorts() 中可直接填写的输入端口；ROS 参数不写入 XML。
# 每项依次声明菜单名称、BT 注册名及端口默认值；None 表示必填。
MISSIONS = (
    ("takeoff", "Takeoff", (("target_alt", "0.5"),)),
    ("nav", "Nav", (
        ("x", None), ("y", None), ("z", None),
        ("tolerance", "0.1"), ("timeout", "60.0"))),
    ("hover", "Hover", (("hover_duration", "1.0"),)),
    ("waitstep", "WaitStep", (("wait_duration", "1.0"),)),
    ("round", "Round", (
        ("center_x", None), ("center_y", None), ("center_z", None),
        ("radius", "0.0"), ("angular_speed", "30.0"),
        ("total_angle", "360.0"), ("tolerance", "0.3"),
        ("timeout", "60.0"), ("ramp_time", "2.0"))),
    ("guidance", "Guidance", (
        ("target_id", None), ("timeout", "30.0"),
        ("tracking_duration", "0.0"))),
)


# 对应节点的 ROS 私有参数位于 config/nav.yaml，和任务端口分别配置。
MISSION_HINTS = {
    "Takeoff": "ROS 配置 takeoff/: ascent_speed、timeout；此处仅填写起飞高度。",
    "Nav": "目标、tolerance、timeout 填写在 XML；速度、加速度、PID 配置见 nav/。",
    "Hover": "hover_duration 为悬停时长，单位 s。",
    "WaitStep": "wait_duration 为等待时长，单位 s。",
    "Round": "圆心、半径、角速度、总角度、容差、超时及 ramp_time 均为 XML 端口。",
    "Guidance": "ROS 配置 guidance/: tolerance、downward_camera、kp；时长填在 XML。",
}

PORT_LABELS = {
    "target_alt": "altitude (target_alt, m)",
    "target_id": "target_id (目标 ID)",
    "timeout": "timeout (s)",
    "tracking_duration": "tracking_duration (s, 0=仅对齐)",
    "hover_duration": "hover_duration (s)",
    "wait_duration": "wait_duration (s)",
    "angular_speed": "angular_speed (deg/s)",
    "total_angle": "total_angle (deg)",
    "ramp_time": "ramp_time (s)",
}


def output_path():
    """源码运行相对包目录定位，catkin 安装后通过包索引定位。"""
    package = Path(__file__).resolve().parents[2]
    if not (package / "package.xml").is_file():
        import rospkg
        package = Path(rospkg.RosPack().get_path("maxt_pkg"))
    return package / "config" / "mission" / "fast_mission.xml"


def read_parameter(mission, port, default):
    label = PORT_LABELS.get(port, port)
    suffix = " [{}]".format(default) if default is not None else " (必填)"
    while True:
        value = input("{} {}{}: ".format(mission, label, suffix)).strip()
        if not value:
            if default is None:
                print("请输入参数。")
                continue
            value = default
        if port == "target_id":
            return value
        if valid_number(value):
            return value
        print("请输入有限数值。")


def build_xml(tasks):
    root = ET.Element("root", main_tree_to_execute="FastMission")
    tree = ET.SubElement(root, "BehaviorTree", ID="FastMission")
    sequence = ET.SubElement(tree, "Sequence")
    ET.SubElement(sequence, "ConnectCheck")
    for tag, parameters in tasks:
        ET.SubElement(sequence, tag, parameters)
    ET.SubElement(sequence, "TouchDown")
    # minidom 的缩进接口兼容 ROS Noetic 使用的 Python 3.8。
    return minidom.parseString(ET.tostring(root)).documentElement.toprettyxml(
        indent="    ")


def save_tasks(tasks):
    destination = output_path()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(build_xml(tasks), encoding="utf-8")
    return destination


class MissionMonitor:
    """ROS 回调只更新快照；终端绘制和编辑始终在主线程进行。"""

    def __init__(self, pose_topic="/mavros/local_position/pose",
                 task_topic="/maxt_test_node/mission_status"):
        self.pose_topic = pose_topic
        self.task_topic = task_topic
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.connection = "正在连接 ROS"
        self.pose = None
        self.pose_at = None
        self.frame = "--"
        self.mission = None
        self.mission_at = None
        self.worker = None

    def start(self):
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def _run(self):
        # 无 ROS 或 master 未启动时仍可编排；网络等待不会阻塞界面。
        try:
            import http.client
            import xmlrpc.client
            import rospy
            from geometry_msgs.msg import PoseStamped
            from std_msgs.msg import String
        except ImportError:
            with self.lock:
                self.connection = "ROS 未加载 · 离线编排"
            return

        class Transport(xmlrpc.client.Transport):
            def make_connection(self, host):
                return http.client.HTTPConnection(host, timeout=1.0)

        subscribers = []
        try:
            while not self.stop.is_set():
                try:
                    with xmlrpc.client.ServerProxy(
                            os.environ.get("ROS_MASTER_URI", "http://localhost:11311"),
                            transport=Transport()) as master:
                        code, _, _ = master.getPid("/fast_mission_monitor")
                    if code == 1:
                        break
                except (OSError, xmlrpc.client.Error):
                    pass
                with self.lock:
                    self.connection = "ROS 未连接 · 离线编排"
                if self.stop.wait(2.0):
                    return
            if self.stop.is_set():
                return
            rospy.init_node("fast_mission_monitor", anonymous=True,
                            disable_signals=True, log_level=rospy.FATAL,
                            argv=["fast_mission_monitor"])
            subscribers = [
                rospy.Subscriber(self.pose_topic, PoseStamped, self.on_pose, queue_size=1),
                rospy.Subscriber(self.task_topic, String, self.on_mission, queue_size=1),
            ]
            with self.lock:
                self.connection = "ROS 已连接"
            while not self.stop.wait(0.2) and not rospy.is_shutdown():
                pass
        except Exception as error:
            with self.lock:
                self.connection = "监看不可用: {}".format(error)
        finally:
            for subscriber in subscribers:
                subscriber.unregister()

    def close(self):
        self.stop.set()

    def on_pose(self, message):
        p, q = message.pose.position, message.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = sum(component * component for component in values[3:])
        if not all(math.isfinite(value) for value in values) or norm < 1e-12:
            return
        # 四元数归一化后的 ENU yaw，展示为角度，不混用工程标定坐标。
        yaw = math.degrees(math.atan2(
            2 * (q.w * q.z + q.x * q.y), norm - 2 * (q.y * q.y + q.z * q.z)))
        with self.lock:
            self.pose = (p.x, p.y, p.z, yaw)
            self.frame = message.header.frame_id or "MAVROS local"
            self.pose_at = time.monotonic()

    def on_mission(self, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                return
            data.setdefault("active", [])
            if not isinstance(data["active"], list):
                return
            if not isinstance(data.get("state"), str):
                return
            if not all(isinstance(node, dict) and isinstance(node.get("name"), str)
                       for node in data["active"]):
                return
        except (ValueError, TypeError):
            return
        with self.lock:
            self.mission = data
            self.mission_at = time.monotonic()
            # latched 快照可能早于订阅时间；用发送时间识别旧的运行状态。
            try:
                self.mission_at -= max(0.0, time.time() - float(data["stamp"]))
            except (KeyError, TypeError, ValueError):
                pass

    def snapshot(self):
        with self.lock:
            return {"connection": self.connection, "pose": self.pose,
                    "pose_age": None if self.pose_at is None else time.monotonic() - self.pose_at,
                    "frame": self.frame, "mission": self.mission,
                    "mission_age": None if self.mission_at is None else time.monotonic() - self.mission_at}


def draw_text(screen, row, column, text, style=0, width=None):
    """按终端列宽裁剪文本，避免中文和窗口缩小时越界。"""
    import unicodedata
    height, columns = screen.getmaxyx()
    available = max(0, columns - column - 1)
    if width is not None:
        available = min(available, width)
    visible = ""
    for char in text:
        size = 2 if unicodedata.east_asian_width(char) in ("W", "F") else 1
        if size > available:
            break
        visible += char
        available -= size
    if 0 <= row < height and visible:
        try:
            screen.addstr(row, column, visible, style)
        except curses.error:
            pass


ACCENT = curses.A_BOLD
MUTED = curses.A_DIM
PANEL = 0
SELECTED = curses.A_REVERSE
INPUT = 0


def init_theme():
    """Claude 陶橙色搭配编辑器深灰背景，低色终端自动降级。"""
    global ACCENT, MUTED, PANEL, SELECTED, INPUT
    ACCENT, MUTED, PANEL, SELECTED, INPUT = curses.A_BOLD, curses.A_DIM, 0, curses.A_REVERSE, 0
    if not curses.has_colors():
        return
    try:
        curses.start_color()
        curses.use_default_colors()
        rich = curses.COLORS >= 256
        orange, background = (173, 235) if rich else (curses.COLOR_YELLOW, -1)
        subdued = 245 if rich else curses.COLOR_WHITE
        field = 237 if rich else -1
        for number, foreground, shade in (
                (1, orange, background), (2, subdued, background),
                (3, curses.COLOR_WHITE, background), (4, orange, field),
                (5, curses.COLOR_WHITE, field)):
            curses.init_pair(number, foreground, shade)
        ACCENT = curses.color_pair(1) | curses.A_BOLD
        MUTED = curses.color_pair(2)
        PANEL = curses.color_pair(3)
        SELECTED = curses.color_pair(4) | curses.A_BOLD
        INPUT = curses.color_pair(5)
    except curses.error:
        pass


def valid_number(value):
    try:
        return math.isfinite(float(value))
    except ValueError:
        return False


class ParameterForm:
    """参数编辑保存在副本中，完成最后一项才提交到编排序列。"""

    def __init__(self, definition, current=None):
        self.name, self.tag, self.ports = definition
        self.values = {port: (current or {}).get(port, default) or ""
                       for port, default in self.ports}
        self.index = 0
        self.cursor = len(self.value)
        self.error = ""

    @property
    def port(self):
        return self.ports[self.index][0]

    @property
    def value(self):
        return self.values[self.port]

    def handle(self, key):
        if key == "\x1b":
            return "cancel"
        if key in ("\n", "\r", curses.KEY_ENTER, "\t", curses.KEY_DOWN):
            value = self.value.strip()
            if not value or (self.port != "target_id" and not valid_number(value)):
                self.error = "请输入目标 ID。" if self.port == "target_id" else "请输入有限数值。"
                return None
            self.values[self.port] = value
            self.error = ""
            if self.index == len(self.ports) - 1:
                for index, (port, _) in enumerate(self.ports):
                    candidate = self.values[port].strip()
                    if not candidate or (port != "target_id" and not valid_number(candidate)):
                        self.index = index
                        self.cursor = len(self.value)
                        self.error = "请完成此参数。"
                        return None
                return "done"
            self.index += 1
            self.cursor = len(self.value)
        elif key in (curses.KEY_UP, curses.KEY_BTAB):
            self.index = max(0, self.index - 1)
            self.cursor = len(self.value)
            self.error = ""
        elif key == curses.KEY_LEFT:
            self.cursor = max(0, self.cursor - 1)
        elif key == curses.KEY_RIGHT:
            self.cursor = min(len(self.value), self.cursor + 1)
        elif key in (curses.KEY_HOME, "\x01"):
            self.cursor = 0
        elif key in (curses.KEY_END, "\x05"):
            self.cursor = len(self.value)
        elif key in (curses.KEY_BACKSPACE, "\x7f", "\b"):
            if self.cursor:
                self.values[self.port] = self.value[:self.cursor - 1] + self.value[self.cursor:]
                self.cursor -= 1
        elif key == curses.KEY_DC:
            self.values[self.port] = self.value[:self.cursor] + self.value[self.cursor + 1:]
        elif key == "\x15":
            self.values[self.port] = ""
            self.cursor = 0
        elif isinstance(key, str) and key.isprintable():
            self.values[self.port] = self.value[:self.cursor] + key + self.value[self.cursor:]
            self.cursor += len(key)
        return None


def monitor_lines(snapshot):
    """把运行数据与编排数据分开；过期快照不能显示成仍在运行。"""
    if not snapshot:
        return ["监看未启用", "--offline 可仅编排", "当前任务: 等待状态", "X  -- m", "Y  -- m", "Z  -- m", "Yaw  -- deg"]
    data = snapshot["mission"]
    age = snapshot["mission_age"]
    lines = [snapshot["connection"]]
    if data is None:
        lines += ["任务: 等待状态", "当前: --"]
    else:
        terminal = data["state"] in ("SUCCESS", "FAILURE", "STOPPED")
        stale = age > 2.0 and not terminal
        lines.append("任务: " + ("状态已过期" if stale else data["state"]))
        active = data["active"]
        prefix = "上次: " if stale or terminal else "当前: "
        lines.append(prefix + (", ".join(
            "{} #{}".format(node["name"], node.get("uid", "?")) for node in active) or "--"))
    pose = snapshot["pose"]
    pose_age = snapshot["pose_age"]
    lines.append("定位: " + ("等待数据" if pose is None else
                              "已过期" if pose_age > 2.0 else "实时"))
    for label, value, unit in zip(("X", "Y", "Z", "Yaw"), pose or (None,) * 4,
                                   ("m", "m", "m", "deg")):
        lines.append("{}  {} {}".format(label, "--" if value is None else "{:.3f}".format(value), unit))
    lines.append("坐标系: " + snapshot["frame"])
    if pose_age is not None:
        lines.append("定位更新: {:.1f}s 前".format(pose_age))
    if data:
        lines.append("任务文件: " + Path(data.get("xml_path", "--")).name)
        lines.append("任务更新: {:.1f}s 前".format(age))
    lines += ["MAVROS 本地坐标 / ENU", "只读监看 · 不控制飞行"]
    return lines


class MissionWorkspace:
    def __init__(self, monitor=None):
        self.tasks = []
        self.catalog_index = 0
        self.task_index = 0
        self.focus = 0  # 任务库、编排序列、运行监看
        self.form = None
        self.edit_index = None
        self.monitor = monitor
        self.monitor_offset = 0
        self.preview = False
        self.preview_offset = 0
        self.dirty = False
        self.status = "就绪 · 选择任务并按 Enter 添加"

    def handle(self, key):
        if self.form:
            outcome = self.form.handle(key)
            if outcome == "done":
                task = (self.form.tag, dict(self.form.values))
                if self.edit_index is None:
                    self.tasks.append(task)
                    self.task_index = len(self.tasks) - 1
                else:
                    self.tasks[self.edit_index] = task
                self.dirty = True
                self.status = "已更新 {} · 尚未保存".format(task[0])
                self.form = None
            elif outcome == "cancel":
                self.form = None
                self.status = "已取消参数编辑"
            return True
        if self.preview:
            if key in ("p", "q", "\x1b"):
                self.preview = False
            elif key in (curses.KEY_UP, curses.KEY_DOWN):
                self.preview_offset = max(0, self.preview_offset + (1 if key == curses.KEY_DOWN else -1))
            return True
        if key == "q":
            return False
        if key in ("\t", curses.KEY_RIGHT, curses.KEY_BTAB, curses.KEY_LEFT):
            self.focus = (self.focus + (-1 if key in (curses.KEY_BTAB, curses.KEY_LEFT) else 1)) % 3
        elif key == "m":
            self.focus = 1 if self.focus == 2 else 2
        elif key in (curses.KEY_UP, curses.KEY_DOWN):
            delta = 1 if key == curses.KEY_DOWN else -1
            if self.focus == 0:
                self.catalog_index = (self.catalog_index + delta) % len(MISSIONS)
            elif self.focus == 1 and self.tasks:
                self.task_index = (self.task_index + delta) % len(self.tasks)
            elif self.focus == 2:
                self.monitor_offset = max(0, self.monitor_offset + delta)
        elif key in ("\n", "\r", curses.KEY_ENTER):
            if self.focus == 0:
                self.edit_index = None
                self.form = ParameterForm(MISSIONS[self.catalog_index])
            elif self.focus == 1 and self.tasks:
                self.edit_index = self.task_index
                tag, parameters = self.tasks[self.task_index]
                self.form = ParameterForm(next(item for item in MISSIONS if item[1] == tag), parameters)
        elif self.focus == 1 and self.tasks and key == "d":
            self.tasks.pop(self.task_index)
            self.task_index = max(0, min(self.task_index, len(self.tasks) - 1))
            self.dirty = True
            self.status = "已删除 · 尚未保存"
        elif self.focus == 1 and self.tasks and key in ("J", "K"):
            target = self.task_index + (1 if key == "J" else -1)
            if 0 <= target < len(self.tasks):
                self.tasks[self.task_index], self.tasks[target] = self.tasks[target], self.tasks[self.task_index]
                self.task_index = target
                self.dirty = True
                self.status = "已调整顺序 · 尚未保存"
        elif key == "p":
            self.preview, self.preview_offset = True, 0
        elif key == "s":
            try:
                self.status = "已保存: {}".format(save_tasks(self.tasks))
                self.dirty = False
            except OSError as error:
                self.status = "保存失败: {}".format(error)
        return True

    def draw_catalog(self, screen, x, top, width, rows):
        draw_text(screen, top, x, "任务库", ACCENT if self.focus == 0 else MUTED, width)
        page = max(1, rows - 1)
        start = max(0, self.catalog_index - page + 1)
        for index in range(start, min(len(MISSIONS), start + page)):
            selected = self.focus == 0 and index == self.catalog_index
            draw_text(screen, top + 1 + index - start, x,
                      ("› " if selected else "  ") + MISSIONS[index][0],
                      SELECTED if selected else PANEL, width)

    def draw_sequence(self, screen, x, top, width, rows):
        draw_text(screen, top, x, "编排序列 · {} 项".format(len(self.tasks)),
                  ACCENT if self.focus == 1 else MUTED, width)
        if rows <= 1:
            return
        if self.preview:
            lines = build_xml(self.tasks).splitlines()
            page = max(1, rows - 1)
            self.preview_offset = min(self.preview_offset, max(0, len(lines) - page))
            for index, line in enumerate(lines[self.preview_offset:self.preview_offset + page]):
                draw_text(screen, top + index + 1, x, line, PANEL, width)
            return
        if rows >= 4:
            draw_text(screen, top + 1, x, "  ConnectCheck  [固定]", MUTED, width)
            draw_text(screen, top + rows - 1, x, "  TouchDown     [固定]", MUTED, width)
            first, page = top + 2, rows - 3
        else:
            first, page = top + 1, rows - 1
        start = max(0, self.task_index - page + 1)
        for index in range(start, min(len(self.tasks), start + page)):
            tag, parameters = self.tasks[index]
            selected = self.focus == 1 and index == self.task_index
            label = "{}{:02d} {}  {}".format("› " if selected else "  ", index + 1, tag,
                                          " ".join("{}={}".format(k, v) for k, v in parameters.items()))
            draw_text(screen, first + index - start, x, label, SELECTED if selected else PANEL, width)
        if not self.tasks:
            draw_text(screen, first, x, "选择任务后按 Enter 添加", MUTED, width)

    def draw_monitor(self, screen, x, top, width, rows):
        draw_text(screen, top, x, "运行监看", ACCENT if self.focus == 2 else MUTED, width)
        lines = monitor_lines(self.monitor.snapshot() if self.monitor else None)
        page = max(1, rows - 1)
        self.monitor_offset = min(self.monitor_offset, max(0, len(lines) - page))
        for index, line in enumerate(lines[self.monitor_offset:self.monitor_offset + page]):
            draw_text(screen, top + 1 + index, x, line, PANEL, width)

    def draw(self, screen):
        screen.erase()
        try:
            screen.bkgd(" ", PANEL)
        except curses.error:
            pass
        height, width = screen.getmaxyx()
        # 所有窗口尺寸都可操作；窄屏一次展示一个区域，Tab / m 切换。
        header = 2 if height >= 9 else (1 if height >= 4 else 0)
        footer = 6 if height >= 12 else (3 if height >= 6 else (1 if height >= 3 else 0))
        rows = max(1, height - header - footer)
        wide = width >= 105 and rows >= 5
        title = " FAST MISSION  /  maxt_pkg"
        if not wide:
            title += "  /  " + ("任务库", "编排序列", "运行监看")[self.focus]
        if header:
            draw_text(screen, 0, 0, title, ACCENT)
        if header >= 2:
            draw_text(screen, 1, 0, " fast_mission.xml{}   ·   {}".format(
                " ●" if self.dirty else "", "XML 预览" if self.preview else "任务编辑器"), MUTED)
        if wide:
            catalog_width, monitor_width = 19, 30
            center = catalog_width + 2
            monitor_x = width - monitor_width
            for y in range(header, header + rows):
                draw_text(screen, y, catalog_width, "│", MUTED)
                draw_text(screen, y, monitor_x - 2, "│", MUTED)
            self.draw_catalog(screen, 1, header, catalog_width - 2, rows)
            self.draw_sequence(screen, center, header, monitor_x - center - 3, rows)
            self.draw_monitor(screen, monitor_x, header, monitor_width - 1, rows)
        else:
            pane = 1 if self.preview else self.focus
            (self.draw_catalog, self.draw_sequence, self.draw_monitor)[pane](
                screen, 0, header, max(1, width - 1), rows)
        if footer:
            self.draw_footer(screen, height - footer, width, footer)
        if self.form and footer == 0:
            draw_text(screen, height - 1, 0, self.form.port + " › " + self.form.value, ACCENT)
        screen.refresh()

    def draw_footer(self, screen, top, width, rows):
        form = self.form
        if rows >= 6:
            draw_text(screen, top, 0, "─" * max(0, width - 1), MUTED)
            label = "参数输入" if form else "参数面板"
            if form:
                label += "  /  {}  /  {} ({}/{})".format(
                    form.name, PORT_LABELS.get(form.port, form.port), form.index + 1, len(form.ports))
            draw_text(screen, top + 1, 1, label, ACCENT)
            draw_text(screen, top + 2, 1, self.input_line(width), INPUT)
            if form:
                hint = form.error or MISSION_HINTS[form.tag]
            else:
                hint = self.status
            draw_text(screen, top + 3, 1, hint, MUTED)
        elif rows >= 3:
            label = form.port + " " if form else ""
            draw_text(screen, top, 0, label + self.input_line(width), INPUT)
            draw_text(screen, top + 1, 0, form.error if form and form.error else self.status, MUTED)
        if form:
            help_text = "Enter/Tab 下一项并提交  ↑ 上一项  Esc 取消  Ctrl+U 清空  ←→ 移动光标"
        elif self.preview:
            help_text = "↑↓ 滚动 XML  ·  p / q / Esc 返回"
        else:
            help_text = "Tab 切换区域  ↑↓ 选择  Enter 添加/编辑  d 删除  J/K 移动"
        if rows >= 6:
            draw_text(screen, top + rows - 2, 1, help_text, MUTED)
        final = "s 保存  p 预览  m 监看  q 退出" if not form else help_text
        if rows == 1 and form:
            final = form.port + " " + self.input_line(width)
        draw_text(screen, top + rows - 1, 0, final, ACCENT)

    def input_line(self, width):
        if not self.form:
            return "› 选中任务后按 Enter，在此填写具体参数"
        form = self.form
        visible = max(1, width - 10)
        start = max(0, form.cursor - visible // 2)
        value = form.value[start:form.cursor] + "▏" + form.value[form.cursor:start + visible]
        return "› " + value


def edit_task(screen, definition, current=None):
    """保留独立编辑入口，使用与工作区相同的参数表单。"""
    workspace = MissionWorkspace()
    workspace.form = ParameterForm(definition, current)
    while workspace.form:
        workspace.draw(screen)
        try:
            key = screen.get_wch()
        except curses.error:
            continue
        workspace.handle(key)
    return workspace.tasks[0] if workspace.tasks else None


def tui(screen, monitor=None):
    screen.keypad(True)
    screen.timeout(100)
    init_theme()
    try:
        curses.curs_set(0)
    except curses.error:
        pass
    workspace = MissionWorkspace(monitor)
    while True:
        workspace.draw(screen)
        try:
            key = screen.get_wch()
        except curses.error:
            continue
        if not workspace.handle(key):
            return 0


def main():
    parser = argparse.ArgumentParser(description="快速 BT XML 任务编排器")
    parser.add_argument("--tui", action="store_true", help="启动终端图形界面")
    parser.add_argument("--offline", action="store_true", help="仅编排，不连接 ROS 监看")
    parser.add_argument("--pose-topic", default="/mavros/local_position/pose", help="监看的 PoseStamped 话题")
    parser.add_argument("--task-topic", default="/maxt_test_node/mission_status", help="监看的任务状态话题")
    args = parser.parse_args()
    if args.tui:
        if not sys.stdin.isatty() or not sys.stdout.isatty():
            parser.error("--tui 需要交互式终端")
        locale.setlocale(locale.LC_ALL, "")
        monitor = None if args.offline else MissionMonitor(args.pose_topic, args.task_topic)
        if monitor:
            monitor.start()
        try:
            return curses.wrapper(tui, monitor)
        finally:
            if monitor:
                monitor.close()
    tasks = []
    print("快速 BT XML 任务编排器（距离 m，时间 s，角度 °）")
    print("参数回车使用默认值；Ctrl+C / Ctrl+D 退出且不保存。")
    print("起降速度与超时、视觉控制参数见 config/nav.yaml；Nav/Round/Guidance 的任务超时仍在 XML 中填写。")
    while True:
        print("\nmission list:")
        for index, (name, _, _) in enumerate(MISSIONS, 1):
            print("{}. {}".format(index, name))
        print("0. 生成并覆盖 fast_mission.xml\nu. 撤销最后一个任务\nq. 退出且不保存")
        choice = input("选择任务（已添加 {} 个）: ".format(len(tasks))).strip().lower()
        if choice == "q":
            return 0
        if choice == "u":
            if tasks:
                print("已撤销 {}".format(tasks.pop()[0]))
            continue
        if choice == "0":
            destination = output_path()
            xml = build_xml(tasks)
            save_tasks(tasks)
            print("\n{}已生成：{}".format(xml, destination))
            return 0
        if not choice.isdigit() or not 1 <= int(choice) <= len(MISSIONS):
            print("请输入菜单中的编号。")
            continue
        name, tag, ports = MISSIONS[int(choice) - 1]
        print(MISSION_HINTS[tag])
        parameters = {
            port: read_parameter(name, port, default) for port, default in ports
        }
        tasks.append((tag, parameters))
        print("已添加：{}".format(ET.tostring(
            ET.Element(tag, parameters), encoding="unicode")))


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (KeyboardInterrupt, EOFError):
        print("\n已取消，未保存。")
        sys.exit(0)
    except OSError as error:
        print("保存失败：{}".format(error), file=sys.stderr)
        sys.exit(1)
