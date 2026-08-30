# maxt_pkg

基于 ROS 1、MAVROS 和 BehaviorTree.CPP v3 的基础无人机飞行任务包。

当前版本只保留连接检查、起飞、航点导航、五次轨迹导航、等待、定点悬停、圆周运动和自主下降。比赛使用的视觉识别、投放机构、降落点选择、圆环感知/穿越以及固定场地标定已经移除。

详细设计见 [基础架构与开发指南](docs/ARCHITECTURE_AND_REWRITE_GUIDE.zh-CN.md)。

## 架构

```text
Behavior Tree XML
        │
        ▼
MaxtCore ──20 Hz tick──► 基础飞行动作节点
        │
        ▼
MavKit ──20 Hz setpoint──► MAVROS ──► 飞控
```

- MaxtCore：注册节点、加载 XML、运行行为树。
- MavKit：保存 MAVROS 状态，调用 arm/mode 服务，持续发布 setpoint。
- MavActionNode：所有基础飞行动作的 StatefulActionNode 基类。

## 可用行为树节点

| 节点 | 作用 |
|---|---|
| ConnectCheck | 等待 MAVROS 连接 |
| Takeoff | 解锁、进入 OFFBOARD、垂直起飞 |
| GoTo | 分段位置目标导航 |
| QuinticNav | 五次多项式 PVA 轨迹导航 |
| WaitStep | 非阻塞等待，不改变当前 setpoint |
| Hover | 记录并保持当前位置 |
| Round | 带五次加减速的圆周运动 |
| TouchDown | 垂直下降、接地确认、上锁 |

## 目录

```text
include/maxt_pkg/       C++ 接口和轨迹/PID 工具
src/maxt_mav/           MAVROS 适配层
src/maxt_nodes/         基础行为树节点
nodes/                  主程序入口
config/mission/example/ 基础任务 XML
launch/                 通用和示例 launch
script/tools/           速度、轨迹监视工具
```

## 构建

依赖 ROS Noetic、MAVROS、BehaviorTree.CPP v3、geometry_msgs 和 tf2。

```bash
cd /home/a/catkin_ws
catkin_make --pkg maxt_pkg
source devel/setup.bash
```

## 运行

运行 2 米往返示例：

```bash
roslaunch maxt_pkg line2.launch
```

运行一圈圆周运动示例：

```bash
roslaunch maxt_pkg round.launch
```

两个示例默认假定 MAVROS 和定位系统已经由外部启动。如需使用仓库原有的外部启动包：

```bash
roslaunch maxt_pkg line2.launch run_mavros:=true run_localization:=true
```

也可以直接使用通用入口：

```bash
roslaunch maxt_pkg mission.launch \
  bt_xml_path:=$(rospack find maxt_pkg)/config/mission/example/line2.xml
```

## MAVROS 接口

订阅：

- /mavros/state
- /mavros/extended_state
- /mavros/local_position/pose
- /mavros/local_position/velocity_local

发布：

- /mavros/setpoint_position/local
- /mavros/setpoint_raw/local

服务：

- /mavros/cmd/arming
- /mavros/set_mode

## 注意

- XML 中的 x/y/z 直接使用 MAVROS 本地坐标系，不再执行比赛场地旋转标定。
- WaitStep 只等待；需要稳定保持位置时使用 Hover。
- 导航末端对齐和降落确认超时会明确返回 FAILURE，不会伪造成功或强制上锁。
- 实飞前必须在仿真或拆桨环境验证坐标轴、OFFBOARD 切换、setpoint 频率和上锁条件。
