# maxt_pkg 基础架构与开发指南

## 1. 当前范围

maxt_pkg 已从比赛任务工程收缩为基础飞行包，只负责：

- MAVROS 连接和飞行状态访问
- 解锁与 OFFBOARD 模式切换
- 起飞
- 位置航行
- 五次多项式轨迹航行
- 逻辑等待和定点悬停
- 圆周运动
- 自主下降、接地确认和上锁

已经移除：

- QR、YOLO、相机及自定义视觉消息
- 投放机构和舵机接口
- 左右降落点选择
- 圆环点云检测、圆环中心接收和穿越任务
- 固定比赛场地任务、模型和仿真外设
- 基于单个场地参考向量的 XY 旋转标定
- 未实现或未使用的 Land、RingPass、RingTarget、XNavigation 和 MinSnap 声明

## 2. 运行架构

```text
mission XML
    │
    ▼
maxt_test_node
    │
    ▼
MaxtCore ──注册与 tick──► Stateful BT Actions
    │                           │
    └───────────────────────────┤
                                ▼
                              MavKit
                                │
                   state / pose │ setpoint / service
                                ▼
                              MAVROS
```

MaxtCore 默认以 20 Hz tick 行为树。MavKit 使用 ROS Timer 以默认 20 Hz 发布最近的 setpoint，因此短暂的 BT tick 抖动不会直接中断 OFFBOARD setpoint 流。

运行线程：

| 上下文 | 工作 |
|---|---|
| 主线程 | tick 行为树 |
| AsyncSpinner 两个线程 | MAVROS 订阅和 Timer 回调 |
| 外部进程 | MAVROS、定位和飞控 |

## 3. 目录

```text
include/maxt_pkg/
├── maxt_core.hpp
├── maxt_mav.hpp
├── maxt_nodes.hpp
├── pid_controller.hpp
├── quintic_curve.hpp
└── maxt_nodes/
    ├── connect_check_node.hpp
    ├── goto_node.hpp
    ├── hover_node.hpp
    ├── quintic_nav_node.hpp
    ├── round_node.hpp
    ├── takeoff_node.hpp
    ├── touch_down_node.hpp
    └── waitstep_node.hpp

src/
├── maxt_core.cpp
├── maxt_mav/
└── maxt_nodes/

config/mission/example/
├── line2.xml
└── round.xml
```

## 4. MavKit

### 4.1 输入接口

| Topic | 类型 |
|---|---|
| /mavros/state | mavros_msgs/State |
| /mavros/extended_state | mavros_msgs/ExtendedState |
| /mavros/local_position/pose | geometry_msgs/PoseStamped |
| /mavros/local_position/velocity_local | geometry_msgs/TwistStamped |

### 4.2 输出接口

| Topic/Service | 类型 |
|---|---|
| /mavros/setpoint_position/local | geometry_msgs/PoseStamped |
| /mavros/setpoint_raw/local | mavros_msgs/PositionTarget |
| /mavros/cmd/arming | mavros_msgs/CommandBool |
| /mavros/set_mode | mavros_msgs/SetMode |

### 4.3 SetpointMode

| 模式 | 行为 |
|---|---|
| STANDBY | 不发布 |
| HEARTBEAT | 发布当前位姿，维持 OFFBOARD |
| CONTROL | 发布目标 PoseStamped |
| RAW_CTRL | 发布 PositionTarget |

内部不再做比赛场地坐标旋转。BT 端口和 MAVROS pose/setpoint 必须使用同一套本地坐标约定。

## 5. 行为树节点

### ConnectCheck

输入：timeout，默认 60 秒。

只等待 MAVROS connected 状态。它不保证 pose 已经到达或定位已经收敛，后续应增加状态新鲜度检查。

### Takeoff

输入：

- target_alt：默认 0.5 m，相对 home Z
- ascent_speed：默认 0.5 m/s
- timeout：默认 30 s

状态机：

```text
ARM → OFFBOARD → INIT → ASCEND → DONE
```

起飞使用位置 XY、速度 Z 和固定 yaw 的混合 PositionTarget。

### GoTo

输入：

- x/y/z：必填
- timeout：30 s
- tolerance：0.3 m
- step：0.5 m

它把目标分成连续小步，每个 tick 在当前位置前方生成一个虚拟位置目标。到达判据是三轴分别进入 tolerance。

### QuinticNav

输入：

- x/y/z：必填
- v_max：2.5 m/s
- a_max：1.2 m/s²
- tolerance：0.1 m
- timeout：60 s
- pid_kp/ki/kd：1.0/0.05/0

每轴采用五次多项式，起终点速度和加速度为零，并同时下发位置、速度和加速度。PID 速度修正叠加在轨迹前馈上。

轨迹结束后最多继续对齐三秒；仍未进入容差时明确返回 FAILURE。

仍需继续修正：

- yaw 没有显式保持，默认目标可能为零
- 未校验零或负的速度、加速度限制
- position-only 末端目标会被同 tick 的 PVA 目标覆盖

### WaitStep

输入 wait_duration，默认 1 秒。

只进行非阻塞计时，不修改 MavKit 的 setpoint。其效果依赖前一动作留下的控制模式。

### Hover

输入 hover_duration，默认 1 秒。

记录启动时位姿和 yaw，在指定时间内持续保持。需要飞行稳定时优先使用 Hover，而不是 WaitStep。

### Round

输入：

- center_x/y/z：必填
- radius：默认 0，自动按当前位置计算
- angular_speed：30 deg/s
- total_angle：360 deg
- tolerance：0.3 m
- timeout：60 s
- ramp_time：2 s

使用五次角度曲线实现加速、匀速、减速，并下发圆周 PVA 目标。

仍需继续修正短圆弧问题：当总角度小于完整加减速所需角度时，当前实现会先超出目标，再跳回最终角度。

### TouchDown

输入：

- descent_speed：-0.5 m/s
- timeout：60 s
- ground_timeout：3 s
- height_tol：0.3 m
- vel_tol：0.2 m/s

下降期间保持 XY，以高度、垂速和 ExtendedState 三条件确认接地，再请求上锁。

只有确认接地后才会请求上锁。总超时会切回 HEARTBEAT 并明确返回 FAILURE。

## 6. 示例任务

line2.xml：

```text
连接 → 起飞 → 悬停 → 前飞 2 m → 悬停 → 返回 → 降落
```

round.xml：

```text
连接 → 起飞 → 到圆周起点 → 悬停 → 绕一圈 → 悬停 → 返回 → 降落
```

示例坐标是教学参数，不代表场地标定结果。

## 7. 构建与运行

```bash
cd /home/a/catkin_ws
catkin_make --pkg maxt_pkg
source devel/setup.bash
```

```bash
roslaunch maxt_pkg line2.launch
roslaunch maxt_pkg round.launch
```

示例 launch 默认不启动外部 MAVROS 和定位。可用 run_mavros、run_localization 参数开启仓库原先依赖的外部 launch。

## 8. 后续重构顺序

1. 为状态增加时间戳新鲜度和 pose-ready 检查。
2. 给 setpoint 增加所有权、有效期和 watchdog。
3. 明确 ENU/NED、frame_id 和 yaw 契约。
4. 为轨迹、PID、Round 短圆弧和节点状态机增加单元测试。
5. 让控制进程在行为树失败后继续维持安全 setpoint，而不是直接退出。

## 9. 实飞前最低检查

- 拆桨验证 arm/mode 请求顺序。
- 检查 OFFBOARD 前 setpoint 预发送是否满足飞控要求。
- 验证 PositionTarget 坐标轴和 type_mask。
- 验证 heartbeat 发布频率及最大间隔。
- 验证定位丢失、MAVROS 断连和行为树 FAILURE。
- 验证只有确认接地后才能 disarm。
- 先执行 line2，再执行 round，最后才增加新动作。
