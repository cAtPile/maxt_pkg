"""检查轻量任务生成器与 C++ 节点端口的兼容性；无需 ROS 运行环境。"""

import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import re
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "fast_mission", PACKAGE / "script/fast_mission/fast_mission.py")
mission = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mission)


def input_ports(source):
    # 只读取端口声明区域，避免把运行阶段的 getInput 或 ROS 参数当成端口。
    declaration = source.split("::providedPorts()", 1)[1].split("\n}", 1)[0]
    ports = {}
    for name, default in re.findall(
            r'BT::InputPort<[^>]+>\(\s*"([^"]+)"\s*(?:,\s*([^,)]+))?',
            declaration):
        ports[name] = default.strip() or None
    return ports


class FastMissionPortsTest(unittest.TestCase):
    def test_input_names_and_defaults_match_nodes(self):
        sources = {"waitstep": "waitstep", "takeoff": "takeoff", "nav": "nav",
                   "hover": "hover", "round": "round", "guidance": "guidance"}
        for name, tag, ports in mission.MISSIONS:
            with self.subTest(node=tag):
                if tag == "RegionTraverse":
                    continue  # 子树接口在独立测试中验证。
                actual = input_ports((PACKAGE / "src/maxt_nodes" /
                                      (sources[name] + ".cpp")).read_text())
                self.assertEqual(dict(ports).keys(), actual.keys())
                for port, default in ports:
                    if default is None:
                        self.assertIsNone(actual[port])
                    else:
                        self.assertEqual(float(default), float(actual[port]))

    def test_generated_xml_has_only_current_ports(self):
        tasks = [(tag, {port: default or "1" for port, default in ports})
                 for _, tag, ports in mission.MISSIONS]
        sequence = ET.fromstring(mission.build_xml(tasks)).find("./BehaviorTree/Sequence")
        self.assertEqual(sequence[0].tag, "ConnectCheck")
        self.assertEqual(sequence[-1].tag, "TouchDown")
        self.assertEqual(sequence[0].attrib, {})
        self.assertEqual(sequence[-1].attrib, {})
        for node, (tag, parameters) in zip(list(sequence)[1:-1], tasks):
            if tag == "RegionTraverse":
                self.assertEqual(node.tag, "SubTreePlus")
                self.assertEqual(node.attrib, {
                    **parameters, "ID": tag,
                    "result": "{region_search_result_%d}" % list(sequence).index(node)})
            else:
                self.assertEqual(node.tag, tag)
                self.assertEqual(node.attrib, parameters)
        self.assertEqual(input_ports((PACKAGE / "src/maxt_nodes/touch_down.cpp").read_text()), {})

    def test_region_subtree_cli_and_repeated_instances(self):
        index, definition = next((i, item) for i, item in enumerate(mission.MISSIONS, 1)
                                 if item[1] == "RegionTraverse")
        values = ["0", "0", "-6", "6", "1", "4"]
        with tempfile.TemporaryDirectory() as folder:
            destination = Path(folder) / "mission.xml"
            with patch("sys.argv", ["fast_mission.py"]), \
                    patch("builtins.input", side_effect=[str(index)] + values + ["0"]), \
                    patch.object(mission, "output_path", return_value=destination), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(mission.main(), 0)
            root = ET.parse(destination).getroot()
            self.assertEqual(root.find("include").get("path"), "subtree/RegionTraverse.xml")
            node = root.find(".//SubTreePlus")
            parameters = dict(zip((port for port, _ in definition[2]), values))
            self.assertEqual(node.attrib, {**parameters, "ID": "RegionTraverse",
                                          "result": "{region_search_result_1}"})
        root = ET.fromstring(mission.build_xml([("RegionTraverse", parameters)] * 2))
        self.assertEqual(len(root.findall("include")), 1)
        nodes = root.findall(".//SubTreePlus")
        self.assertNotEqual(nodes[0].get("result"), nodes[1].get("result"))
        subtree = ET.parse(PACKAGE / "config/mission/subtree/RegionTraverse.xml")
        planner = subtree.find(".//TraversePathGenerator")
        for port in parameters:
            expected = "{%s}" % port
            if port == "altitude":
                self.assertEqual(subtree.find(".//Nav").get("z"), expected)
            else:
                self.assertEqual(planner.get(port), expected)
        self.assertIsNone(ET.fromstring(mission.build_xml([])).find("include"))

    def test_cli_prompts_and_saves_current_guidance_ports(self):
        index = next(i for i, item in enumerate(mission.MISSIONS, 1)
                     if item[1] == "Guidance")
        with tempfile.TemporaryDirectory() as folder:
            destination = Path(folder) / "mission.xml"
            with patch("sys.argv", ["fast_mission.py"]), \
                    patch("builtins.input", side_effect=[str(index), "marker_0", "45", "10", "0"]), \
                    patch.object(mission, "output_path", return_value=destination), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(mission.main(), 0)
            node = ET.parse(destination).find(".//Guidance")
            self.assertEqual(node.attrib, {
                "target_id": "marker_0", "timeout": "45", "tracking_duration": "10"})

    def test_tui_uses_same_guidance_inputs(self):
        class Screen:
            def __init__(self):
                self.keys = iter(list("marker_0") + ["\n", "\n", "\n"])

            def getmaxyx(self):
                return 12, 60

            def erase(self):
                pass

            def bkgd(self, *args):
                pass

            def addstr(self, *args):
                pass

            def refresh(self):
                pass

            def get_wch(self):
                return next(self.keys)

        definition = next(item for item in mission.MISSIONS if item[1] == "Guidance")
        self.assertEqual(mission.edit_task(Screen(), definition), (
            "Guidance", {"target_id": "marker_0", "timeout": "30.0", "tracking_duration": "0.0"}))


class MissionWorkspaceTest(unittest.TestCase):
    def test_edit_is_committed_only_after_last_parameter(self):
        workspace = mission.MissionWorkspace()
        workspace.handle("\n")
        workspace.handle("\x15")
        workspace.handle("2")
        workspace.handle("\n")
        self.assertEqual(workspace.tasks, [("Takeoff", {"target_alt": "2"})])
        workspace.handle("\t")
        workspace.handle("\n")
        workspace.handle("\x15")
        workspace.handle("5")
        workspace.handle("\x1b")
        self.assertEqual(workspace.tasks[0][1]["target_alt"], "2")

    def test_backtracking_cannot_commit_an_empty_required_field(self):
        definition = next(item for item in mission.MISSIONS if item[1] == "Nav")
        form = mission.ParameterForm(definition, {
            "x": "1", "y": "2", "z": "3", "tolerance": "0.1", "timeout": "60"})
        form.handle("\n")
        form.handle("\x15")
        form.handle(mission.curses.KEY_UP)
        form.handle("\n")
        self.assertIsNone(form.handle("\n"))
        self.assertTrue(form.error)

    def test_layout_keeps_input_and_sidebar_in_the_window(self):
        class Screen:
            def __init__(self, size):
                self.size = size
                self.drawn = []

            def getmaxyx(self):
                return self.size

            def erase(self):
                self.drawn = []

            def bkgd(self, *args):
                pass

            def addstr(self, y, x, text, *args):
                self.drawn.append((y, x, text))

            def refresh(self):
                pass

        workspace = mission.MissionWorkspace()
        for size in ((1, 1), (3, 12), (8, 35), (16, 105), (30, 140)):
            for focus in range(3):
                screen = Screen(size)
                workspace.focus = focus
                workspace.draw(screen)
                for y, x, _ in screen.drawn:
                    self.assertTrue(0 <= y < size[0] and 0 <= x < size[1])
            workspace.form = mission.ParameterForm(mission.MISSIONS[0])
            workspace.draw(screen)
            if size[0] >= 8:
                self.assertTrue(any("0.5" in line[2] for line in screen.drawn))
            workspace.form = None
        self.assertTrue(any("运行监看" in line[2] for line in screen.drawn))


class MissionMonitorTest(unittest.TestCase):
    def test_pose_yaw_and_stale_status(self):
        monitor = mission.MissionMonitor()
        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            pose=SimpleNamespace(position=SimpleNamespace(x=1, y=2, z=3),
                                 orientation=SimpleNamespace(x=0, y=0, z=math.sqrt(0.5), w=math.sqrt(0.5))))
        monitor.on_pose(message)
        self.assertAlmostEqual(monitor.snapshot()["pose"][3], 90)
        monitor.pose_at -= 3
        monitor.on_mission(SimpleNamespace(data=json.dumps({
            "state": "RUNNING", "stamp": time.time() - 5,
            "active": [{"name": "Guidance", "uid": "12"}, {"name": "UgvCowork", "uid": "13"}]})))
        lines = mission.monitor_lines(monitor.snapshot())
        self.assertIn("任务: 状态已过期", lines)
        self.assertIn("定位: 已过期", lines)
        self.assertTrue(any("Guidance #12, UgvCowork #13" in line for line in lines))
        monitor.on_mission(SimpleNamespace(data='{"state": "SUCCESS"}'))
        self.assertIn("任务: SUCCESS", mission.monitor_lines(monitor.snapshot()))
        monitor.on_mission(SimpleNamespace(data='{"active": 1}'))
        self.assertEqual(monitor.snapshot()["mission"]["state"], "SUCCESS")


if __name__ == "__main__":
    unittest.main()
