---
name: mavt-dev
description: >
  MAVT (Mavros eXtra function based on Behavior Tree) 无人机自主任务系统开发。
  触发场景：添加新的BT行为树节点（Action/Condition），编写/修改任务XML，
  添加Python脚本，修改CMakeLists.txt构建，使用MavKit API，
  注册节点到MaxtCore，创建launch文件。
---

# MAVT 项目开发指南

## 项目架构

三层架构：`[BT XML 任务层] -> [MaxtCore 核心层] -> [MavKit 硬件抽象层]`

- **MavKit** (`include/maxt_pkg/maxt_mav.hpp`, `src/maxt_mav/`): 线程安全的 MAVROS 封装，提供飞控指令和状态查询
- **MaxtCore** (`include/maxt_pkg/maxt_core.hpp`, `src/maxt_core.cpp`): BT 工厂注册、节点管理、主循环 tick
- **BT Nodes** (`include/maxt_pkg/maxt_nodes/`, `src/maxt_nodes/`): 行为树节点实现
- **Python 脚本** (`script/`): QR检测、YOLO分类、相机驱动、可视化工具
- **Launch 文件** (`launch/`): 模块化启动配置
- **任务 XML** (`config/mission/`): 行为树任务定义

## 开发工作流

### 1. 添加新的 BT Action 节点（需要 MavKit 访问）

需要修改/创建的文件（按顺序）：

**Step 1 — 创建头文件** `include/maxt_pkg/maxt_nodes/<name>_node.hpp`:

```cpp
#ifndef MAXT_NODES_<NAME>_HPP
#define MAXT_NODES_<NAME>_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class <Name>Node : public MavActionNode {
public:
    <Name>Node(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    enum class Phase : uint8_t { <PHASES> } phase_;
    // 输入参数
    // 中间状态
    ros::Time start_time_;
};

} // namespace maxt
#endif
```

**Step 2 — 创建实现文件** `src/maxt_nodes/<name>.cpp`:

遵循 `StatefulActionNode` 生命周期：
- `providedPorts()`: 声明 `BT::InputPort` / `BT::OutputPort`
- 构造函数: 转发给 `MavActionNode(name, config, mav)`
- `onStart()`: 读取 input port 参数 → 初始化 phase → 记录 `start_time_` → 返回 `RUNNING`
- `onRunning()`: `switch(phase_)` 状态机，每 tick 执行一步，完成后返回 `SUCCESS`/`FAILURE`
- `onHalted()`: 清理，通常调用 `mav_.setSetpointMode(SetpointMode::HEARTBEAT)`

参考文件: `src/maxt_nodes/takeoff.cpp` (takeoff 是标准范例)

**Step 3 — 添加到 CMakeLists.txt**:

在 `add_library(${PROJECT_NAME}` 块中添加:
```cmake
src/maxt_nodes/<name>.cpp
```

**Step 4 — 在 MaxtCore 中注册** (`src/maxt_core.cpp`):

添加 include:
```cpp
#include <maxt_pkg/maxt_nodes/<name>_node.hpp>
```

在 `registerNodes()` 方法中添加注册:
```cpp
factory_.registerBuilder<<Name>Node>("<Name>",
    [this](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<<Name>Node>(name, config, *mav_);
    });
```

**Step 5 — 在 XML 中声明和使用**: 见下方「编写任务 XML」部分。

### 2. 添加新的 BT Condition 节点（无需 MavKit）

条件节点实现更简单：

- 继承 `BT::ConditionNode`（不是 `MavActionNode`）
- 只需实现 `static providedPorts()` 和 `BT::NodeStatus tick() override`
- 注册时不传 `*mav_`: `factory_.registerBuilder<X>("X", [](auto& n, auto& c) { return std::make_unique<X>(n, c); })`

参考文件: `src/maxt_nodes/conditions.cpp`, `src/maxt_nodes/target_check2.cpp`

### 3. 编写任务 XML

位置: `config/mission/<场景>/<name>.xml`

结构：
```xml
<root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
        <Sequence>
            <!-- 按顺序排列 BT 节点 -->
            <ConnectCheck timeout="30" />
            <Takeoff target_alt="1.3" ascent_speed="0.5" timeout="20" />
            <QuinticNav x="1.8" y="0.0" z="1.2" timeout="30" tolerance="0.1" v_max="2.0" a_max="1.0" />
            <TouchDown descent_speed="-0.5" timeout="60" ground_timeout="3.0" />
        </Sequence>
    </BehaviorTree>

    <TreeNodesModel>
        <Action ID="ConnectCheck">
            <parameter name="timeout" type="double" default="60.0" />
        </Action>
        <!-- 每个使用的节点都要在此声明参数 -->
    </TreeNodesModel>
</root>
```

要点：
- XML 属性名必须与 `providedPorts()` 中的 `InputPort` 名称精确匹配
- 参数类型: `double`, `string`, `int`
- 控制流节点: `<Sequence>` (顺序执行), `<Fallback>` (选择/容错), `<Parallel>`
- 每个使用的节点都必须在 `<TreeNodesModel>` 中声明

参考文件: `config/mission/example/4d.xml`

### 4. 添加 Python 脚本

- 放在 `script/` 目录
- 首行 shebang 指向解释器（可能需要虚拟环境路径，如 `#!/home/a/yolov5_venv/bin/python3`）
- 标准 ROS 节点模式: `rospy.init_node()` → 订阅/发布 → `rospy.spin()`
- 如果需要在 launch 中控制启用，参考现有脚本的参数化模式

### 5. 修改 Launch 文件

主 launch 文件: `launch/mission.launch`

添加新的可选组件模式：
```xml
<arg name="run_<feature>" default="false" />
<node if="$(arg run_<feature>)" pkg="maxt_pkg" type="<script>.py" name="<node_name>" output="screen" />
```

## 关键约定

### 命名规范
- BT 节点类名: `PascalCase`，以 `Node` 结尾 (如 `TakeoffNode`)
- XML 标签 ID: 与类名去掉 `Node` 后缀一致 (如 `Takeoff`)
- 头文件: `snake_case_node.hpp` (如 `takeoff_node.hpp`)
- 实现文件: `snake_case.cpp` (如 `takeoff.cpp`)
- 所有代码在 `namespace maxt {}` 内

### 线程安全
- MavKit 所有数据成员受 `std::mutex` 保护
- BT 节点在主线程运行 (tick)，ROS 回调在 AsyncSpinner 线程
- 节点通过 MavKit 的公开查询方法访问状态（返回副本，线程安全）

### Setpoint 模式
- `HEARTBEAT`: 维持 OFFBOARD，发布当前位置（保活）
- `CONTROL`: 位置控制模式
- `RAW_CTRL`: 速度+位置混合控制（用于轨迹跟随）
- `STANDBY`: 静默

节点结束时务必切回 `HEARTBEAT`，否则失去 OFFBOARD 连接。

### 降落检测
使用三重投票：高度接近 home + 垂速接近零 + `extended_state.landed_state == ON_GROUND`，需持续确认 `ground_timeout` 秒后才 disarm。

## Gotchas

- **不要** 在 `onStart()` 中返回 `SUCCESS`/`FAILURE`，`StatefulActionNode` 只应返回 `RUNNING`
- **务必** 在 `onHalted()` 中调用 `mav_.setSetpointMode(SetpointMode::HEARTBEAT)` 清理
- **不要** 在 BT 节点中做阻塞操作（sleep、忙等），使用 phase 状态机分步执行
- **不要** 忘记在 XML 的 `<TreeNodesModel>` 中声明新节点及其参数
- **不要** 忘记在 `CMakeLists.txt` 中添加新的 `.cpp` 文件
- **不要** 忘记在 `maxt_core.cpp` 中添加 `#include` 和 `registerBuilder`
- 使用 `ros::Time::now()` 而非系统时间进行超时检测
- `PositionTarget` 的 `type_mask` 使用 `IGNORE_*` 标志选择控制通道（位置/速度/加速度/偏航）
- 首次发布 topic 时有延迟，重要 publisher 需在 `MaxtCore::init()` 中预建立
- servo 名称: `front_left`, `front_right`, `back_left`, `back_right`，对应 topic `/servo/<name>/open` 和 `/servo/<name>/close`

## 构建与测试

```bash
# 构建
cd ~/catkin_ws && catkin build maxt_pkg

# 模拟测试
roslaunch maxt_pkg s_forresult.launch

# 单节点测试
roslaunch maxt_pkg t_fr.launch
```

## 参考文件索引

- MavKit API 详情 → `references/mavkit-api.md`
- BT 节点实现详细模式 → `references/bt-node-pattern.md`
- XML 编写详细模式 → `references/xml-pattern.md`
