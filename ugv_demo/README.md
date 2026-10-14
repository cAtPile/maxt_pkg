# ROS1 控制 PX4 小车 Demo

本目录属于 `maxt_pkg`，无需另建嵌套 catkin 包。适用于 ROS Noetic + MAVROS，以及支持 MAVLink Rover 速度 Offboard 的 PX4 固件（接口以 PX4 v1.14 为参考）。

## 准备

- 启动 PX4 **Rover** SITL 或连接已配置好的 PX4 小车；不要启动本项目无人机的 `sim.launch`。
- 小车需要有效的速度/姿态估计，GPS 或外部定位配置应满足所用固件的解锁条件。
- SITL 模型名称随 PX4 版本变化。若使用含 Gazebo Classic `rover` 目标的旧版本，可在 PX4 源码目录运行 `make px4_sitl gazebo-classic_rover`；以该版本实际提供的构建目标为准。此 demo 不安装或启动 PX4/Gazebo。
- 优先在仿真中验证。启动节点会自动申请 OFFBOARD 和解锁。不要同时运行无人机控制节点或其他 setpoint 发布者。

## 运行

### 同时启动 Gazebo 可视化仿真

本机已有 `/home/a/Libraries/PX4-Autopilot` 的 Rover 模型、SITL 可执行文件和 Gazebo 插件，可一键启动 Gazebo GUI、PX4 Rover、MAVROS 和控制节点：

```bash
~/catkin_ws/src/maxt_pkg/ugv_demo/start_sim.sh
# 只看仿真，不自动解锁行驶：
~/catkin_ws/src/maxt_pkg/ugv_demo/start_sim.sh run_control:=false
# 调整行驶参数：
~/catkin_ws/src/maxt_pkg/ugv_demo/start_sim.sh goal_x:=3 goal_y:=3 speed:=0.3 tolerance:=0.2
```

通过 `PX4_DIR=/path/to/PX4-Autopilot` 可指定另一个兼容的 PX4 源码目录。该入口对接本机的 Gazebo Classic / `Tools/sitl_gazebo` 布局。默认加载空地面和 `rover` 小车，`gui:=false` 可关闭窗口，`world:=/absolute/path/to/file.world` 可替换场景。启动前关闭占用相同端口的其他 PX4/Gazebo 仿真。终端 Ctrl+C 停止整套仿真。

若环境已配置好，也可直接运行 `roslaunch maxt_pkg ugv_sim.launch`。

### 连接已启动的 PX4

源码目录可直接启动，无需先编译此 Python 节点：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch ~/catkin_ws/src/maxt_pkg/ugv_demo/ugv_demo.launch run_mavros:=true
```

已有 MAVROS 时省略 `run_mavros:=true`。串口示例：

```bash
roslaunch ~/catkin_ws/src/maxt_pkg/ugv_demo/ugv_demo.launch \
  run_mavros:=true fcu_url:=/dev/ttyACM0:57600 speed:=0.3 goal_x:=3 goal_y:=3
```

需要安装空间时重新构建本包；launch 安装到包的 `ugv_demo` 目录：

```bash
cd ~/catkin_ws
catkin_make --pkg maxt_pkg
source devel/setup.bash
roslaunch maxt_pkg ugv_demo.launch speed:=0.5 goal_x:=3 goal_y:=3
```

默认先连续发送 2 秒零速度，再请求 OFFBOARD、解锁，读取 `/mavros/local_position/pose`，闭环驶向局部 ENU 坐标 `(3,3)`。目标是定位坐标系的绝对坐标，不是相对当前位置，也不保证与 Gazebo 世界原点重合。最大速度为 `speed`（默认 0.5 m/s），接近目标时按距离减速；XY 欧氏距离不超过 `tolerance`（默认 0.2 m）时记录 `Goal reached`，锁定为零速度。停止指令后的实际漂移需在仿真中验证。

到点模式最长行驶 120 秒；超时后同样停止，但不会记录到达成功。位置反馈超过 1 秒未更新时停止。MAVROS 自动做 ENU→NED 转换。

本机旧 PX4 的 `RoverPositionControl::control_velocity()` 在 `_velocity_frame == NED` 分支直接把世界速度用于车身转向，未扣除当前航向，导致普通世界速度控制绕圈。默认 `legacy_rover_velocity:=true` 利用位置话题的姿态反馈进行航向补偿，编码经 MAVROS 转换后的车身前/右速度分量。这是针对本机固件的兼容处理，不能视为标准 NED 速度接口；换用已修正此问题的固件时设置 `legacy_rover_velocity:=false`。两个模式均需新鲜的姿态反馈才能执行此补偿。

保留原定时直行模式：`./ugv_demo/start_sim.sh use_goal:=false speed:=0.5 duration:=5 heading_deg:=0`。此时 `heading_deg=0` 指向东/+X，`90` 指向北/+Y；这是世界坐标方向，不是车头前方。`duration` 和 `heading_deg` 只用于定时模式。

## 接口与停止

| 接口 | 用途 |
| --- | --- |
| `/mavros/state` | 连接、模式、解锁反馈 |
| `/mavros/local_position/pose` | 到点控制的位置反馈（ENU） |
| `/mavros/setpoint_raw/local` | 20 Hz `PositionTarget`，仅启用 vx/vy/vz |
| `/mavros/set_mode` | 请求 OFFBOARD |
| `/mavros/cmd/arming` | 请求解锁 |

`mavros_ns` 可用于已有的独立小车 MAVROS 实例；本 launch 自带的 MAVROS 使用默认 `/mavros` 命名空间。

行驶期间状态超过 2 秒未更新、断连、退出 OFFBOARD 或上锁，节点锁定为零速度，不会自动恢复行驶或重新抢占模式；重启节点才会再次执行。完成后小车仍保持解锁。可用遥控器切换模式，或确认停稳后调用：

```bash
rosservice call /mavros/cmd/arming "value: false"
```

Ctrl+C 时尽力发送一次零速度；进程终止和链路断开无法保证指令送达，需配置并验证 PX4 的 Offboard 丢失保护。节点启动等待超时为 60 秒。模式/解锁服务返回成功仍以 `/mavros/state` 的实际状态为准。

排查时使用 `rostopic echo /mavros/state` 与 `rostopic hz /mavros/setpoint_raw/local`，并检查 PX4/QGroundControl 的拒绝原因。确认运行的是 Rover 固件、定位有效、端口正确及无其他控制节点。

参考：[PX4 v1.14 Rover Offboard 支持](https://docs.px4.io/v1.14/en/flight_modes/offboard#rover)。PX4 v1.17 文档注明 Rover 不支持 MAVLink Offboard API、支持 ROS2，不能假设本 ROS1 demo 通用于所有版本：[v1.17 文档](https://docs.px4.io/v1.17/en/flight_modes/offboard#rover)。

## Action 移动协同 demo

按 `docs/cowork-ugv_guide.md` 实现，控制端可作为后续无人机任务模块的调用示例。这里只实现受控 UGV 移动，未实现无人机编队、ReadyCheck 或投放。

首次使用先编译生成 Action：

```bash
cd ~/catkin_ws
catkin_make --pkg maxt_pkg
source devel/setup.bash
```

两个终端分别执行（先启动仿真，不启动旧的速度控制节点）：

```bash
# 终端 1，在 maxt_pkg 目录
./ugv_demo/start_sim.sh run_control:=false
# 终端 2，加载工作空间环境后
roslaunch maxt_pkg ugv_move.launch
```

默认发送 `(3,3)`。配置文件 `ugv_demo/config/ugv_move.yaml` 的 `profile` 选择 `profiles` 中的目标列表，每个目标成功后才发送下一个；失败即停止。可以编辑配置，或使用：

```bash
roslaunch maxt_pkg ugv_move.launch profile:=two_points
roslaunch maxt_pkg ugv_move.launch config:=/absolute/path/custom.yaml
```

目标是 MAVROS 本地 ENU 绝对坐标，单位米。服务端复用原 demo 的 20Hz 速度流、到点减速和本机 PX4 航向补偿。默认最大速度 0.5m/s、容差 0.2m，每个移动任务超时 120 秒，启动等待超时 60 秒；超时、反馈丢失、退出 OFFBOARD 或取消均停止发送非零速度，结果为失败/取消。到点后仍保持解锁和零速度。

Action 名称默认 `/ugv_mover`，类型 `maxt_pkg/ugv_moverAction`：goal 为 `target_x/target_y`，result 为 `succeed`，feedback 的 `state` 为 `WAITING=0`、`STARTING=1`、`MOVING=2`、`REACHED=3`。失败说明保存在 actionlib 的状态文本中。新目标抢占当前目标；完成后服务端继续等待下一条任务。指南中的 `MOVINGING/STARING` 规范为 `MOVING/STARTING`，`.pu` 按 `.py` 实现。

接入其他协同控制端时，使用 `run_controller:=false` 只启动服务端。观察及取消任务：

```bash
rostopic echo /ugv_mover/feedback
rostopic echo /ugv_mover/result
rostopic pub -1 /ugv_mover/cancel actionlib_msgs/GoalID '{}'
```

同一 MAVROS 实例只能运行一个速度控制节点；启动此 demo 前应停止旧 `ugv_offboard_demo`。运行与 Gazebo 验证由使用者执行。

如果旧版本 Action 服务端在初始化前退出，请更新 `ugv_move_demo.py` 后重启 `ugv_move.launch`。已修复 catkin devel 包装脚本下同目录控制模块的导入路径；此项 Python 修复无需重新编译。
