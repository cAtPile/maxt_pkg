# RegionTraverse 交付说明

## 修改内容

新增同步 BT 节点 `RegionTraverseNode`，在 MaxtCore 注册为 `RegionTraverse`，加入 CMake 库构建。新增完整起飞、遍历和降落的 XML 示例，更新 README。未修改指南，未移除或更改既有节点接口。

## 接口

输入均为 double：`reg_point_1_x`、`reg_point_1_y`，依次到 `reg_point_4_x`、`reg_point_4_y`，以及 `coverage_step`。

四角必须按矩形边界顺时针或逆时针排列，坐标单位为米，使用同一个平面坐标系；支持旋转矩形。依照项目准则，调用方负责保证输入为合法矩形。

按用户更正，输入端口由 `coverage_area` 改为 `coverage_step`，单位为米，表示相邻两条平行轨迹的间距，不再作面积或平方根换算。原端口不再提供。

输出端口 `waypoints` 类型为 `maxt::RegionWaypoints`，即 `std::vector<std::array<double, 2>>`，每项为 `[x, y]`；`waypoint_count` 类型为 `int`。XML 示例分别映射到黑板 `region_waypoints` 和 `region_waypoint_count`。C++ 消费者需包含 `region_traverse_node.hpp` 并按上述类型读取；数组不是字符串。

## 设计

航带沿点 1 到点 2 的方向，沿点 1 到点 4 的方向推进。航带数为 `max(1, ceil(边14长度 / coverage_step))`；相邻航带间距严格为 coverage_step，剩余宽度平均留在两侧。首条航带距边界 `(边14长度 - (航带数 - 1) * coverage_step) / 2`。每条航带两个端点，相邻航带方向交替，得到蛇形路线。间距大于或等于区域横向宽度时，仅生成中心航带的两个端点。

每次 tick 重新读取输入并覆盖输出，成功返回 SUCCESS；缺少输入、非正或非有限间距、退化横向边长以及超过 100000 条航带时返回 FAILURE，失败不更新输出。后续节点应仅在成功后读取结果。节点不依赖 MavKit，不发送设点，不执行航点飞行，也不提供高度或航向。

## 验证

使用本机 ROS Noetic 和 BehaviorTree.CPP v3 编译新实现，并通过实际工厂加载 XML、tick 和黑板读取断言验证：10×6 米矩形、间距 2 米生成 `(0,1), (10,1), (10,3), (0,3), (0,5), (10,5)`，数量为 6；旋转矩形、大间距生成中心航带的两个端点；重复 tick 改为零间距返回 FAILURE。

未进行飞行仿真或硬件验证；本模块仅规划二维路径。

## 三条核心建议

1. 根据传感器覆盖宽度与所需重叠率设置 coverage_step，并将旧 XML 的 coverage_area 改为新端口。
2. 让后续执行节点读取同一数组类型，并显式设置安全高度和导航速度。
3. 实际飞行前检查角点顺序、坐标系、障碍物与安全边界；当前路线不包含避障规划。

## 完整遍历任务更新

`region_traverse_test.xml` 改为连接检查、起飞到 4m、规划 6×6m 矩形（间距 1m）、Repeat 循环导航和降落。共 6 条航带、12 个航点，y 从 0.5 到 5.5，x 在 0 与 6 之间交替。最后在 `(0,5.5)` 降落。

新增同步节点 `GetRegionWaypoint`，在 MaxtCore 注册。输入 `waypoints: RegionWaypoints`、`index: int`，输出 `x/y: double`、`next_index: int`。XML 先通过 SetBlackboard 将索引清零，再让 index 和 next_index 映射到同一个黑板键。Repeat 次数来自规划输出的 waypoint_count，每轮 Sequence 取点后执行 QuinticNav，等待导航成功再开始下一轮。原有 QuinticNav 接口不变。补齐规划节点读取输入的代码，避免使用未初始化的角点和间距。

验证：使用真实规划、取点、SetBlackboard、Repeat 和 Sequence 节点加载实际任务 XML，将飞行节点替换为断言桩，确认起飞高度 4m、12 次导航坐标及顺序、导航高度 4m、最后才降落；核心注册代码通过编译语法检查。未进行仿真或硬件飞行验证。

本次三条核心建议：

1. 先在仿真确认 4m 高度和整个 6×6m 区域的飞行空间。
2. 当前正常完成后在末航点降落，请确认该处适合着陆。
3. 当前 Sequence 在导航失败时停止；如需失败时降落，后续应明确增加失败处置分支。

## Launch 入口

新增 `launch/region_traverse_test.launch`，复用 `mission.launch`，将 bt_xml_path 指向 `config/mission/region_traverse_test.xml`。执行 `roslaunch maxt_pkg region_traverse_test.launch` 即启动完整区域遍历任务。沿用 run_mavros、run_fast_lio（默认 false）和 bt_tick_rate、heartbeat_rate（默认 20.0）参数，无既有接口移除。现有 CMake 的 launch 目录安装规则已覆盖此文件。

验证：使用 roslaunch 配置加载检查默认参数及启用外部组件、修改频率的参数传递，不启动飞行进程。

使用建议：

1. 默认先准备好 MAVROS 和定位系统，再启动本任务。
2. 需要入口启动外部组件时，设置 run_mavros:=true 或 run_fast_lio:=true，并确保对应 tutorial 包可用。
3. 首次运行先在仿真核对航迹与末航点着陆位置。
