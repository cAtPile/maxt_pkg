# Mavros eXtra function based on Behavior Tree

版本模式：在线

## 项目概述

`maxt_pkg` 是基于 BehaviorTree.CPP v3 实现的无人机自主任务系统。通过 XML 行为树编排起飞、降落、航点导航、绕圈飞行、QR 码检测、YOLO 目标识别、圆环穿越和条件着陆选择等功能。

## 架构

```
[行为树层]   XML 驱动的任务逻辑
    ↓
[核心层]     MaxtCore: 工厂注册、节点管理、主循环
    ↓
[MAV层]     MavKit: 线程安全的 MAVROS 抽象层
```

- **MavKit** — 封装 MAVROS 通信（状态/位姿订阅、设点发布、arm/mode 服务调用），通过 `std::mutex` 保证线程安全
- **MaxtCore** — 系统编排器：初始化 MavKit、注册所有 BT 节点、加载 XML 行为树、以 20Hz 主循环 tick
- **行为树节点** — 每个节点继承 `BT::StatefulActionNode`，通过 `onStart()/onRunning()/onHalted()` 生命周期实现非阻塞异步动作

## 目录结构

```
maxt_pkg/
├── CMakeLists.txt                  # C++14, 构建 libmaxt_pkg + 2 个可执行文件
├── package.xml                     # BSD, 依赖 mavros/behaviortree_cpp_v3/tf2 等
├── config/                         # 行为树 XML 配置文件
│   ├── mission/                    # 任务配置文件目录
│   ├── template.xml                # 任务配置文件模板示例（包含基本节点的定义，但树是空白的）
│   ├── all.xml                     # 全程任务配置文件示例，包含所有功能（暂时省略）
│   ├── sim/                        # 模拟任务配置文件目录，只能模拟不能实际执行
│   │   ├── sim_all.xml             # 按赛题的全任务流程
│   │   ├── sim_<task_name>.xml     # 其他仿真测试
|   |   └── sim_vision.xml          # 视觉测试QR\YOLO\Deliver,不起飞
│   └── example/                    # 示例任务配置文件目录,可以试飞
│       ├── line_1.xml              # 直线飞行
│       ├── round.xml               # 绕圈飞行
│       ├── ring_detect.xml         # 检测圆环并飞行到圆环前方
│       ├── ring_pass.xml           # 圆环穿越
│       └── vision_test.xml         # 视觉测试QR\YOLO\Deliver
├── include/
│   ├── 

├── launch/
│   ├── maxt_test.launch            # 完整任务启动
│   ├── goto_test.launch
│   ├── qrDetect_test.launch
│   ├── ringPass_test.launch
│   ├── round_test.launch
│   ├── yolo_test.launch
│   └── simple_camera_driver.launch
├── log/                            # 日志目录（空）
├── model/
│   └── best.pt                     # YOLOv5 分类模型权重
├── msg/
│   └── StringStamped.msg           # 自定义消息: Header + string[]
├── nodes/
│   └── maxt_test_node.cpp          # 主节点入口
├── script/
│   ├── qr_detector.py              # QR 码检测节点
│   ├── simple_camera_driver.py     # 摄像头驱动节点
│   ├── yolov5_predict.py           # YOLOv5 目标分类节点
│   └── tools/
│       ├── xvel_monitor.py         # 速度可视化工具
│       └── xytraj_monitor.py       # XY 轨迹可视化工具
└── src/
    ├── maxt_core.cpp               # MaxtCore 实现
    ├── maxt_mav/
    │   ├── maxt_mav_cb.cpp         # MavKit 回调：状态/位姿订阅、心跳定时器
    │   ├── maxt_mav_con.cpp        # MavKit 构造与初始化
    │   └── maxt_mav_request.cpp    # MavKit 服务调用：arm/mode/setpoint
    ├── maxt_nodes/
    │   ├── maxt_nodes_conditions.cpp     # CheckLandLeft / CheckLandRight
    │   ├── maxt_nodes_connect_check.cpp  # ConnectCheck
    │   ├── maxt_nodes_goto.cpp           # GoTo（带分轴 P 控制）
    │   ├── maxt_nodes_land.cpp           # Land
    │   ├── maxt_nodes_pass_ring.cpp      # RingPass（圆环穿越）
    │   ├── maxt_nodes_qr_detect.cpp      # QRDetect
    │   ├── maxt_nodes_round.cpp          # Round（绕圈飞行）
        │   ├── maxt_nodes_takeoff.cpp        # Takeoff
    │   └── maxt_nodes_target_check.cpp   # TargetCheck（YOLO 目标校验）
    └── ring_center_detect_node.cpp       # 圆环中心检测节点（独立可执行文件）
```

## 行为树节点

所有节点通过 `MavKit&` 引用与无人机交互，通过 BT 黑板共享数据。

### 动作节点

| 节点               | 输入端口                                                                                               | 输出端口                                             | 功能                                  |
| ---------------- | -------------------------------------------------------------------------------------------------- | ------------------------------------------------ | ----------------------------------- |
| **ConnectCheck** | `timeout` (60s)                                                                                    | —                                                | 等待 MAVROS 连接建立                      |
| **Takeoff**      | `target_alt` (1.5m), `timeout` (20s)                                                               | —                                                | 解锁→OFFBOARD→爬升至目标高度                 |
| **GoTo**         | `x`, `y`, `z`, `timeout` (30s), `tolerance` (0.3m)                                                 | —                                                | 分轴 P 控制飞至目标点                        |
| **Land**         | `timeout` (60s)                                                                                    | —                                                | 触发 AUTO.LAND，等待落地                   |
| **WaitStep**     | `wait_duration` (1s)                                                                               | —                                                | 非阻塞延时                               |
| **Round**        | `center_x`, `center_y`, `center_z`, `radius` (自动计算), `angular_speed` (30°/s), `total_angle` (360°) | —                                                | 绕中心点圆形轨迹飞行                          |
| **QRDetect**     | `timeout` (5s)                                                                                     | `qr_data`, `target_1`, `target_2`, `land_target` | 订阅 `/qr_detect_result`，解析逗号分隔数据写入黑板 |
| **TargetCheck**  | `timeout` (10s)                                                                                    | `target_found`                                   | 开启 YOLO 检测，比对检测结果与黑板中的目标            |
| **RingPass**     | `timeout` (30s), `tolerance` (0.3m)                                                                | —                                                | 采集点云检测圆环中心→三阶段穿越                    |

### 条件节点

| 节点                 | 功能                              |
| ------------------ | ------------------------------- |
| **CheckLandLeft**  | 黑板 `land_target == "left"` 时成功  |
| **CheckLandRight** | 黑板 `land_target == "right"` 时成功 |

## ROS 接口

### 订阅的 Topic

| Topic                                   | 类型                            | 订阅者                           |
| --------------------------------------- | ----------------------------- | ----------------------------- |
| `/mavros/state`                         | `mavros_msgs::State`          | MavKit                        |
| `/mavros/local_position/pose`           | `geometry_msgs::PoseStamped`  | MavKit                        |
| `/mavros/local_position/velocity_local` | `geometry_msgs::TwistStamped` | xvel\_monitor                 |
| `camera/image_raw`                      | `sensor_msgs::Image`          | qr\_detector, yolov5\_predict |
| `/cloud_registered`                     | `sensor_msgs::PointCloud2`    | ring\_center\_detect\_node    |

### 发布的 Topic

| Topic                             | 类型                           | 发布者                        |
| --------------------------------- | ---------------------------- | -------------------------- |
| `/mavros/setpoint_position/local` | `geometry_msgs::PoseStamped` | MavKit（心跳定时器）              |
| `qr_detect_result`                | `maxt_pkg::StringStamped`    | qr\_detector.py            |
| `yolo_detect`                     | `maxt_pkg::StringStamped`    | yolov5\_predict.py         |
| `/ring_center`                    | `geometry_msgs::PoseStamped` | ring\_center\_detect\_node |
| `camera/image_raw`                | `sensor_msgs::Image`         | simple\_camera\_driver     |

### 服务

| 服务                    | 类型                         | 角色                                     |
| --------------------- | -------------------------- | -------------------------------------- |
| `/mavros/cmd/arming`  | `mavros_msgs::CommandBool` | 客户端（MavKit）                            |
| `/mavros/set_mode`    | `mavros_msgs::SetMode`     | 客户端（MavKit）                            |
| `/toggle_yolo_detect` | `std_srvs::Empty`          | 服务端（yolov5\_predict）/ 客户端（TargetCheck） |

## 自定义消息

**`StringStamped.msg`**：`std_msgs/Header header` + `string[] data`

## 启动文件

| Launch 文件                     | 运行内容                                                                           | 用途          |
| ----------------------------- | ------------------------------------------------------------------------------ | ----------- |
| `maxt_test.launch`            | maxt\_test\_node + qr\_detector                                                | 完整任务        |
| `goto_test.launch`            | maxt\_test\_node                                                               | 航点飞行测试      |
| `qrDetect_test.launch`        | maxt\_test\_node + qr\_detector                                                | QR 检测测试     |
| `round_test.launch`           | maxt\_test\_node                                                               | 绕圈飞行测试      |
| `ringPass_test.launch`        | maxt\_test\_node + ring\_center\_detect\_node                                  | 圆环穿越测试      |
| `yolo_test.launch`            | maxt\_test\_node + yolov5\_predict + qr\_detector + simple\_camera\_driver(可选) | YOLO 目标检测测试 |
| `simple_camera_driver.launch` | simple\_camera\_driver                                                         | 摄像头驱动       |

## ROS 参数

### MaxtCore

| 参数             | 类型     | 默认值  | 说明              |
| -------------- | ------ | ---- | --------------- |
| `bt_xml_path`  | string | —    | 行为树 XML 文件路径    |
| `bt_tick_rate` | double | 20.0 | BT tick 频率 (Hz) |

### MavKit

| 参数                   | 类型     | 默认值  | 说明                 |
| -------------------- | ------ | ---- | ------------------ |
| `position_tolerance` | double | 0.3  | 到达判定距离 (m)         |
| `heartbeat_rate`     | double | 20.0 | OFFBOARD 心跳频率 (Hz) |

### ring\_center\_detect\_node

| 参数                 | 类型     | 默认值       | 说明         |
| ------------------ | ------ | --------- | ---------- |
| `x_min/max`        | double | 4.8/7.2   | 点云 X 轴滤波范围 |
| `y_min/max`        | double | -2.0/-1.2 | 点云 Y 轴滤波范围 |
| `z_min/max`        | double | 1.15/1.35 | 点云 Z 轴滤波范围 |
| `max_history_size` | int    | 10        | 时域平滑历史帧数   |

### yolov5\_predict

| 参数             | 类型     | 默认值             | 说明          |
| -------------- | ------ | --------------- | ----------- |
| `yolov5_root`  | string | —               | YOLOv5 仓库路径 |
| `weights_path` | string | `model/best.pt` | 模型权重路径      |
| `device`       | string | cpu             | 推理设备        |
| `confidence`   | double | 0.85            | 检测置信度阈值     |
| `always_on`    | bool   | false           | 是否始终开启      |

## 设计要点

1. **非阻塞 BT 节点**：全部使用 `StatefulActionNode` 生命周期，避免阻塞 BT 执行
2. **Setpoint 三模式**：`HEARTBEAT`（维持 OFFBOARD）→ `CONTROL`（发布目标位姿）→ `STANDBY`（静默）
3. **线程安全**：MavKit 中所有状态访问由 `std::mutex` 保护，查询返回副本
4. **分轴 P 控制**：GoTo 节点对每个轴独立 P 控制，步长限幅防止超调
5. **黑板数据共享**：QR 检测结果写入黑板，条件节点和 TargetCheck 读取，实现节点间解耦通信

## 依赖

- ROS Melodic/Noetic
- MAVROS (`mavros`, `mavros_msgs`)
- BehaviorTree.CPP v3
- OpenCV (Python: `pyzbar`, `torch`)
- PCL (`pcl_ros`, `pcl_conversions`)
- tf2
