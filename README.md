# Mavros eXtra function based on Behavior Tree

`maxt_pkg` 是基于 ROS 1、MAVROS 和 BehaviorTree.CPP v3 的无人机任务编排包，支持基础飞行、航点导航、圆周运动、区域搜索、ArUco 视觉引导及空地协同。

`MaxtCore` 加载 XML 并驱动行为树；`MavKit` 封装 MAVROS 状态、定位、设点发布、解锁及模式切换。Python 脚本提供相机采集、视觉反馈、小车轻量仿真和轨迹监视。Gazebo 原生插件负责移动靶标。

| 目录 | 内容 |
| --- | --- |
| `include/maxt_pkg/`、`src/` | 核心、MAVROS 接口、行为树节点及 Gazebo 插件 |
| `nodes/` | C++ 任务执行入口 |
| `script/`、`script/tools/` | Python 节点及绘图工具 |
| `launch/` | 任务、识别和仿真启动文件 |
| `config/mission/` | 行为树 XML |
| `config/` | 导航、识别、靶标及小车 YAML 配置 |
| `worlds/`、`models/` | Gazebo 场景与本地模型 |
| `test/` | 空地协同行为树流程测试 |

## 部署

### 环境要求

本工程面向以下环境，仓库未锁定各依赖的补丁版本：

| 组件 | 版本或要求 |
| --- | --- |
| 操作系统 | Ubuntu 20.04 |
| ROS | ROS 1 Noetic，catkin 工作空间 |
| Python | 系统 Python 3.8，使用与 ROS `cv_bridge` 匹配的环境 |
| C++ | 支持 C++17 的编译器；CMake 同时保留了 `-std=c++14` 选项，实际标准以构建命令为准 |
| 行为树 | `behaviortree_cpp_v3`，不是 v4 |
| 飞控接口 | MAVROS、`mavros_msgs`，飞行时需要连接飞控或 SITL 并有有效定位 |
| 基础库 | Boost.System、Eigen3、ROS 消息及 Action 工具链 |
| 仿真 | Gazebo Classic 11 及开发库；当前 CMake 无条件编译 Gazebo 插件，因此基础构建也需要 Gazebo 开发依赖 |
| Python 功能 | rospy、actionlib、rospkg、PyYAML、Matplotlib、OpenCV、cv_bridge；ArUco 识别需要 `cv2.aruco` |
| PX4 仿真（可选） | 外部 `px4`、`mavlink_sitl_gazebo`，包含 `posix_sitl.launch` 和 `iris_downward_depth_camera` 模型 |

PX4 的具体版本未在本仓库锁定；本包不包含 PX4 源码或 SITL 构建产物。Matplotlib 和图像窗口需要桌面显示环境。

### 部署方法

以下假设 ROS Noetic 已安装，包位于 `~/catkin_ws/src/maxt_pkg`。

1. 安装依赖。在已配置 ROS 软件源的机器上执行：

   ```bash
   source /opt/ros/noetic/setup.bash
   sudo apt update
   sudo apt install python3-rosdep libboost-system-dev libeigen3-dev \
     libgazebo11-dev ros-noetic-behaviortree-cpp-v3 \
     ros-noetic-mavros ros-noetic-mavros-extras \
     ros-noetic-cv-bridge ros-noetic-rqt-image-view \
     ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins \
     python3-opencv python3-matplotlib python3-yaml python3-rospkg
   ```

   如果尚未初始化 rosdep，先执行一次 `sudo rosdep init`，再执行：

   ```bash
   rosdep update
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

   MAVROS 所需的 GeographicLib 数据可通过其安装脚本准备：

   ```bash
   sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
   ```

2. 编译并加载工作空间。编译会生成 `Guidance` 和 `ugv_mover` Action 消息：

   ```bash
   cd ~/catkin_ws
   source /opt/ros/noetic/setup.bash
   catkin_make --pkg maxt_pkg
   source devel/setup.bash
   rospack find maxt_pkg
   python3 -c "import cv2; print(cv2.__version__); print(hasattr(cv2, 'aruco'))"
   ```

   最后一项应输出 `True`，否则当前 Python 的 OpenCV 缺少 ArUco 模块。每个新终端都需要加载 ROS 和工作空间环境。直接 `rosrun` 时还需要已运行的 `roscore`。

3. 按用途准备运行环境：

   - 仅运行小车轻量仿真：不需要 PX4 或 MAVROS。
   - 仅查看靶标场景：运行 `sim_world.launch`，不包含无人机。
   - 无人机仿真：先在外部完成 PX4 SITL 配置和构建，确保 `rospack find px4`、`rospack find mavlink_sitl_gazebo` 可解析，然后运行 `sim.launch`。
   - 实机任务：先启动 MAVROS 和定位系统，再启动任务入口。任务 XML 会实际请求解锁、OFFBOARD 和飞行，执行前需核对坐标、高度及运动范围。

`mission.launch` 的可选组件依赖外部包：`run_mavros` 使用 `tutorial_basic/launch/mavros.launch`，`run_fast_lio` 使用 `tutorial_navigation/launch/mid360.launch`，`run_camera` 使用 `tutorial_vision/launch/simple_camera_driver.launch`。这些包不随本项目提供，三个开关默认均为 `false`。

## 使用方法

### nodes

#### 快速任务编排：fast_mission.py

无需启动 ROS，即可用 Python 3 交互式生成顺序行为树：

```bash
python3 script/fast_mission/fast_mission.py
# 编译并加载工作空间后也可运行：
rosrun maxt_pkg fast_mission.py
```

添加 `--tui` 可启动终端界面，无需第三方 Python 依赖，支持 SSH 终端，界面随窗口尺寸自动调整：

```bash
python3 script/fast_mission/fast_mission.py --tui
# 或：rosrun maxt_pkg fast_mission.py --tui
```

界面采用 Claude 陶橙色主题和 VS Code 式工作区：左侧任务库选择 `takeoff/nav/hover` 等任务，中间展示编排序列，右侧显示运行监看，下方参数输入框用于填写具体参数，底栏说明快捷键。支持 256 色的终端使用橙色与深灰背景，低色或无色终端自动降级。窗口较小时自动切换单栏，Tab 在任务库、编排序列和监看之间切换，`m` 直接切换到监看，列表随选中项滚动。

上下键选择任务，Enter 添加或编辑；编排序列中 `d` 删除、`J/K` 下移或上移。参数输入框支持左右键移动光标、Backspace/Delete 删除、Ctrl+U 清空；Enter/Tab 确认当前参数并进入下一项，最后一项确认后提交任务，↑ 返回上一项，Esc 放弃本次编辑。`p` 在中央面板预览 XML，`s` 覆盖保存，`q` 退出。保存后可以继续编辑，每次启动从空序列开始，退出丢弃尚未保存的编辑；连接检查与降落固定在序列首尾。

TUI 默认后台连接 ROS 进行只读监看，未加载 ROS 或 master 尚未启动时仍可编排；使用 `--offline` 完全关闭连接。右栏订阅 `/mavros/local_position/pose` 展示 XYZ（m）、偏航角（deg）及消息坐标系，坐标来自 MAVROS 本地 ENU，未应用 MavKit 的工程坐标标定。数据超过 2 秒没有更新会标记过期。通过 `--pose-topic` 和 `--task-topic` 可指定其他定位或任务状态话题。

当前任务来自执行器发布的 `/maxt_test_node/mission_status`（`std_msgs/String`，JSON），包含 `state`、`xml_path`、墙钟 `stamp`，以及可选的 `active` 节点数组（`uid/name/type`）；没有运行节点时省略 `active`。支持显示并行运行的多个动作，与编辑器选中项分别显示。需要重新编译并加载工作空间后，执行器才会发布该话题；旧执行器没有此话题时显示“等待状态”。TUI 保留本次连接期间收到的最终状态，执行器退出后再打开界面无法恢复上一次结果。此侧栏为后续完整任务监看器预留，当前不提供任务控制或历史回放。

按编号添加 `takeoff`、`nav`、`hover`、`waitstep`、`round` 或 `guidance`，逐项输入端口参数，回车使用显示的默认值。可重复添加同一任务；`u` 撤销最后一项，`q` 或 Ctrl+C / Ctrl+D 退出且不保存。距离单位为米、时间为秒，圆周角度为度；`guidance` 的误差容差使用像素。

生成器按当前 `providedPorts()` 填写任务输入，命令行和 TUI 共用同一份端口定义：

| 任务 | 可填写的 XML 输入端口 |
| --- | --- |
| `takeoff` | `target_alt` |
| `nav` | `x`、`y`、`z`、`tolerance`、`timeout` |
| `hover` | `hover_duration` |
| `waitstep` | `wait_duration` |
| `round` | `center_x/y/z`、`radius`、`angular_speed`、`total_angle`、`tolerance`、`timeout`、`ramp_time` |
| `guidance` | `target_id`、`timeout`、`tracking_duration`（0 表示仅对齐） |

`Takeoff` 的上升速度及超时、`ConnectCheck` 的超时、`TouchDown` 的全部控制参数，以及 `Guidance` 的 `tolerance/downward_camera/kp` 从 `config/nav.yaml` 对应模块读取，不再作为这些节点的 XML 属性生成。`Nav`、`Round` 和 `Guidance` 的任务 `timeout` 仍是 BT 输入端口。`Guidance.message` 为输出端口，当前生成器不要求填写。运行 `python3 -B -m unittest discover -s test -p 'test_fast_mission.py'` 可检查生成器定义、端口默认值和 XML 是否与节点源码一致。

输入 `0` 后覆盖包内 `config/mission/fast_mission.xml`，树名固定为 `FastMission`。脚本始终保留首项 `<ConnectCheck />` 和末项 `<TouchDown />`，中间按添加顺序生成 XML。脚本只生成文件；执行任务使用：

```bash
roslaunch maxt_pkg fast_mission.launch
```

`fast_mission.launch` 自动加载 `config/nav.yaml` 并执行最近保存的快速任务。默认复用已启动的 MAVROS、定位和相机；支持 `nav_config`、`bt_tick_rate`、`heartbeat_rate` 及 `run_mavros/run_fast_lio/run_camera` 参数，与 `mission.launch` 一致。

该顺序树中，前面的任务失败会使整棵树失败，后续降落节点也不会执行。

下面的命令均在已加载工作空间的终端执行。`rosrun` 不会自动加载 launch 中的 YAML，也不会启动依赖节点。

#### 任务执行：maxt_test_node

运行自定义行为树时，需要指定 XML，并将导航配置加载至该节点的私有命名空间：

```bash
rosparam load "$(rospack find maxt_pkg)/config/nav.yaml" /maxt_test_node
rosrun maxt_pkg maxt_test_node \
  _bt_xml_path:="$(rospack find maxt_pkg)/config/mission/navigation_test.xml" \
  _bt_tick_rate:=20.0 _heartbeat_rate:=20.0
```

通常直接使用 `mission.launch` 更方便，它会完成参数加载。`Nav` 的速度、加速度及 PID 参数来自私有 `nav/` 参数，而非 XML 输入端口。

当前注册的行为树节点为：`ConnectCheck`、`CapturePose`、`Takeoff`、`Nav`、`WaitStep`、`Hover`、`Round`、`TouchDown`、`Guidance`、`RegionTraverse`、`GetRegionWaypoint`、`FoundCheck`、`UgvCowork`。这些是 XML 节点，不是独立的 `rosrun` 可执行文件。`Land`、`XNavigation` 未注册。

#### 视觉识别：aruco_center.py

识别指定字典和 ID 的 ArUco，发布中心像素坐标、检测状态与标注图像；可选提供 `/guidance` Action 服务。

```bash
rosparam load "$(rospack find maxt_pkg)/config/aruco_center.yaml" /aruco_center
rosrun maxt_pkg aruco_center.py _run_guidance:=true
```

默认 YAML 输入为 `/camera/rgb/image_raw`，字典为 `DICT_4X4_50`、ID 为 `0`。输出为 `/aruco_center/center`（`PointStamped`，x/y 为像素坐标）、`/aruco_center/is_found`（`Bool`）、`/aruco_center/debug_image`（`Image`）。标注图像仅在有订阅者时发布。使用 USB 相机时将 `_image_topic:=/camera/image_raw` 加到命令末尾。

#### 相机采集：simple_camera_driver.py

```bash
rosrun maxt_pkg simple_camera_driver.py _camera_id:=0 _rate:=10 _frame_id:=mono_camera
```

通过 OpenCV 读取本机相机，发布 `camera/image_raw`（根命名空间下为 `/camera/image_raw`）。它与仿真相机的 `/camera/rgb/image_raw` 不同，识别节点需要使用相同的输入话题。

#### 模拟视觉反馈：guidance_test_server.py

```bash
rosrun maxt_pkg guidance_test_server.py _error_u:=20.0 _duration:=10.0 _rate:=20.0
```

在 `/guidance` 提供固定 `error_u`、`error_v=0` 的反馈，默认持续 10 秒后停止反馈，并等待客户端取消。它不根据图像或无人机运动更新误差，用于验证等待、超时等流程。不要与真实识别节点同时提供同名 Action 服务。

#### 小车仿真与目标发送：ugv_sim.py、ugv_controller.py

先启动 `roscore`，再分别在两个终端执行：

```bash
rosrun maxt_pkg ugv_sim.py
```

```bash
rosrun maxt_pkg ugv_controller.py
```

`ugv_sim.py` 从 `(0,0)` 出发，提供 `/ugv_mover` Action，以恒定速度直线移动，并在 Matplotlib 中绘制路径。默认读取 `config/ugv_confg.yaml`，速度为 `1.0 m/s`；文件名保留 `confg` 拼写。

`ugv_controller.py` 按 `config/ugv_target.yaml` 中的绝对 XY 目标依次发送任务；失败或被抢占后停止，Ctrl+C 取消当前目标。两者均支持 `_config:=/绝对路径/配置.yaml` 和 `_action_name:=/自定义服务名`，服务名必须一致。新目标会抢占旧目标，关闭绘图窗口会退出仿真。单独运行时保持 `/use_sim_time=false`，仿真运动以墙钟推进。

#### 运动监视：xvel_monitor.py、xytraj_monitor.py

```bash
rosrun maxt_pkg xvel_monitor.py \
  _velocity_topic:=/mavros/local_position/velocity_local _time_window:=10.0
rosrun maxt_pkg xytraj_monitor.py _pose_topic:=/mavros/local_position/pose
```

两个命令分别打开速度和 XY 轨迹窗口，实际使用时放在独立终端。轨迹工具还支持 `_x_range:='[-1, 8]'`、`_y_range:='[-3, 3]'` 调整坐标轴范围。

#### 旧版靶标控制：moving_target.py

当前场景由原生 Gazebo 插件驱动，无需另启此脚本。仅在需要旧版 ROS 服务控制时使用：

```bash
roslaunch maxt_pkg sim_world.launch move_target:=false
```

另一个终端运行：

```bash
rosrun maxt_pkg moving_target.py _x:=-4.0 _y:=4.0 _amplitude:=2.0 _period:=25.13
```

脚本等待 `/gazebo/set_model_state`，按仿真时钟沿 Y 轴正弦运动。若 `/moving_target/enabled` 为 true，会拒绝启动以避免与插件竞争。该脚本读取自己的私有参数，不会自动使用 `config/sim_world.yaml`。

#### 行为树流程测试：ugv_cowork_tree_test

```bash
rosrun maxt_pkg ugv_cowork_tree_test \
  "$(rospack find maxt_pkg)/config/mission/ugv_cowork_test.xml"
```

仅在构建时启用 `CATKIN_ENABLE_TESTING` 后生成。该程序用替身节点检查找到目标、未找到、对齐失败、协同失败、返航失败等流程，不连接飞控，也不验证实际飞行动力学或视觉精度。

草稿列出的 `ugv_move_demo.py`、`ugv_move_controller.py`、`ugv_offboard.py` 在当前仓库中不存在，不作为本包部署或使用入口。

### launchs

#### 入口一览

| 启动文件 | 功能与默认行为 |
| --- | --- |
| `mission.launch` | 通用任务入口，必须传入 `bt_xml_path`，自动加载导航配置 |
| `fast_mission.launch` | 执行 `config/mission/fast_mission.xml`，自动加载导航配置 |
| `basic_test.launch` | 起飞 1m，导航至 `(-2,2,1)`，悬停 60s 后降落 |
| `navigation_test.launch` | 起飞 1m，依次导航至 `(0,0,1)`、`(2,0,1)`、`(2,2,1)`、`(0,0,1)` 并悬停，最后降落 |
| `round_test.launch` | 起飞 1m，绕圆心 `(1,0,1)`、半径 1m 顺时针一圈，角速度 20°/s，返回 `(0,0,1)` 后降落 |
| `aruco_center.launch` | 加载识别配置，默认不提供视觉 Action；`run_guidance:=true` 可开启 |
| `sim_world.launch` | 仅 Gazebo 场景和运动靶标，不启动 PX4、MAVROS 或无人机相机 |
| `sim.launch` | 启动 PX4 SITL、Gazebo、MAVROS 和机身下视相机 |
| `guidance_test.launch` | 模拟视觉反馈和飞行任务，默认复用已有仿真 |
| `vision_test.launch` | 默认启动完整仿真、真实 ArUco 识别、图像窗口和视觉跟踪任务 |
| `region_traverse_test.launch` | 默认启动识别和视觉 Action，执行区域搜索及找到后的跟踪 |
| `ugv_cowork_test.launch` | 默认启动识别、视觉 Action、小车轻量仿真，执行协同及返航任务 |

#### 基础任务与自定义 XML

先启动 MAVROS 和定位系统，或在独立终端启动仿真：

```bash
roslaunch maxt_pkg sim.launch
```

然后在另一个终端选择一个任务执行：

```bash
roslaunch maxt_pkg basic_test.launch
# 或使用自定义 XML：
roslaunch maxt_pkg mission.launch bt_xml_path:=/absolute/path/to/task.xml
```

`mission.launch` 支持 `nav_config`（默认 `config/nav.yaml`，包含所有任务共用的无人机参数）、`bt_tick_rate` 和 `heartbeat_rate`（均默认 20Hz），以及前述三个外部组件开关。基础、导航和圆周测试只转发 `run_mavros`、`run_fast_lio` 及两个频率参数；需要自定义导航配置或启动相机时可直接使用 `mission.launch`。

航点使用当前 MavKit 坐标系的绝对坐标，单位为米；默认连接检查不进行坐标标定。起飞高度和航点 Z 的含义需结合定位原点核对。`Nav` 末端等待超过 3s 可能强制返回 SUCCESS，任务成功日志不能单独证明航点误差达标。

#### 场景及视觉跟踪

```bash
# 只查看靶标场景：
roslaunch maxt_pkg sim_world.launch
# 或启动完整视觉飞行测试（与上面的场景二选一）：
roslaunch maxt_pkg vision_test.launch
```

完整测试起飞至 2m，导航至 `(-2,2,2)`，悬停 2s，跟踪 ArUco ID 0 共 60s，最后降落。已有完整仿真时传入 `run_sim:=false`；不需要图像窗口时传入 `view_image:=false`。`vision_config` 可指定另一份识别 YAML。统一配置 `config/nav.yaml` 中 Guidance 的 `kp=0.006`，启用下视相机坐标转换，总超时为 90s。

`sim.launch` 默认飞控机型为 `iris`，Gazebo 模型为 `iris_downward_depth_camera`，相机彩色图像为 `/camera/rgb/image_raw`。可查看：

```bash
rosrun rqt_image_view rqt_image_view /camera/rgb/image_raw
```

`sim.launch`、`sim_world.launch` 均支持 `config:=/绝对路径/自定义.yaml`、`move_target`、`gui` 等参数。靶标配置在插件加载时读取，修改后需重启仿真。

#### 模拟反馈测试

```bash
roslaunch maxt_pkg guidance_test.launch run_sim:=true error_u:=20.0 duration:=10.0 rate:=20.0
```

已有仿真时省略 `run_sim:=true`。该入口提供模拟反馈，不启动真实图像识别。

#### 区域搜索

```bash
roslaunch maxt_pkg region_traverse_test.launch
```

需要外部提供 MAVROS、定位和相机。任务起飞到 4m，为 `(0,0)、(-6,0)、(-6,6)、(0,6)` 的矩形以 1m 间距生成 12 个蛇形航点。`FoundCheck` 检查 `/aruco_center/is_found`；检测为 true 且消息未过期时中断导航并跟踪 60s，然后降落。遍历完成仍未找到则直接降落。搜索导航失败会使任务失败，不进入后续降落。

支持 `run_detection`、`run_guidance`（均默认 true）、`detection_config`、`run_camera`、`run_mavros`、`run_fast_lio` 及两个频率参数。已有识别服务时使用 `run_detection:=false`；跟踪仍需要可用的 `/guidance` 服务。更改识别 ID 或状态话题时，需同步修改 XML 的 Guidance 或 FoundCheck。

#### 空地协同与返航

```bash
roslaunch maxt_pkg ugv_cowork_test.launch
```

先准备无人机仿真或 MAVROS、定位和图像输入。任务记录起飞位置，起飞并搜索，发现标记后先对齐，再并行执行视觉伺服与小车跟随；小车到达后返回起飞点上方并降落。小车直接跟随无人机 XY，要求双方坐标系一致。

已有小车 Action 服务时传入 `run_ugv_sim:=false`；其余感知与外部组件参数与区域搜索入口一致。不要同时运行 `ugv_controller.py`，它发送的目标会抢占协同任务。未找到、搜索失败、对齐失败或协同失败均转入返航；返航使用 `Nav`，失败时结束任务并保持位置，不进入降落。`Nav` 末端等待超过 3s 可能强制成功，因此返航成功不保证位置误差达标。最终 SUCCESS 表示返航降落完成，协同结果以黑板 `cowork_result` 为准。

### 常用配置

| 文件 | 当前关键值与用途 |
| --- | --- |
| [config/nav.yaml](config/nav.yaml) | `v_max=5 m/s`、`a_max=1 m/s²`；PID 为 `1.0/0.05/0.0`；减速时间 1s |
| [config/aruco_center.yaml](config/aruco_center.yaml) | 仿真相机输入、`DICT_4X4_50`、ID 0、检测及图像输出话题 |
| [config/sim_world.yaml](config/sim_world.yaml) | 靶标中心 `(-4,4,0.02)`，Y 轴振幅 2m，最大速度 0.5m/s |
| [config/ugv_confg.yaml](config/ugv_confg.yaml) | 小车恒定速度 1m/s |
| [config/ugv_target.yaml](config/ugv_target.yaml) | 小车依次前往 `(3,0)`、`(3,3)`、`(0,0)` |

靶标沿 Y 轴正弦往返，当前范围为 `[2,6] m`，周期由 `T=2π×amplitude/speed` 计算，约为 25.13s；`speed=0` 时静止。YAML 在 launch 参数之后加载，因此同名位置、振幅参数以 YAML 为准，`speed` 优先于旧 `period`。当前靶标中心与视觉任务的初始航点 `(-2,2,2)` 不同，如需保证进入相机视野，应同步调整场景配置或任务航点。

Python 节点通常在启动时读取配置，修改后需重启对应节点；修改 C++ 或 Action 后需重新编译并加载工作空间。本说明依据仓库源码与配置整理，未在本次文档修改中执行安装、编译或飞行测试。

## 开发

### 文件结构

开发时主要修改以下文件，任务配置、行为实现和底层通信分别维护：

```text
maxt_pkg/
├── nodes/maxt_test_node.cpp          # 创建 MaxtCore，初始化并运行任务
├── include/maxt_pkg/
│   ├── maxt_core.hpp                # 行为树调度核心
│   ├── maxt_mav.hpp                 # MavKit、SetpointMode、CurveState
│   ├── maxt_nodes.hpp               # 飞行动作基类 MavActionNode
│   ├── maxt_nodes/                  # 各行为树节点声明及 RegionWaypoints 类型
│   ├── quintic_curve.hpp            # 三轴五次曲线
│   ├── pid_controller.hpp           # 通用 PID 控制器
│   └── calibrate_tools.hpp          # 世界坐标与定位坐标之间的 XY 旋转
├── src/
│   ├── maxt_core.cpp                # 节点注册、XML 加载、tick 主循环
│   ├── maxt_mav/
│   │   ├── maxt_mav_con.cpp         # 通信初始化、控制模式切换、标定
│   │   ├── maxt_mav_cb.cpp          # 状态回调、设点定时器、状态查询
│   │   └── maxt_mav_request.cpp     # 解锁/模式请求、曲线与减速目标构造
│   ├── maxt_nodes/                  # 各行为树节点实现
│   └── sim/moving_target_plugin.cpp # Gazebo 原生运动插件
├── action/                         # Guidance.action、ugv_mover.action
├── config/mission/                 # 任务编排 XML
├── config/                         # ROS 参数 YAML
├── launch/                         # 参数加载与进程组合
├── script/                         # 感知、相机、小车和辅助工具
└── test/ugv_cowork_tree_test.cpp     # 使用替身节点验证协同任务流程
```

现有节点多数直接位于 `maxt_nodes/` 下，少量历史头文件没有对应的已注册实现。判断节点是否可用，应同时查看 [src/maxt_core.cpp](src/maxt_core.cpp) 的注册代码和 [CMakeLists.txt](CMakeLists.txt) 的编译源列表。

### 接口列表

#### ROS 话题与服务

下表方向以 MavKit 或本包节点为主体；MAVROS 接口在源码中使用绝对路径，切换无人机命名空间时需 remap 或修改接口配置。

| 接口 | 类型 | 方向与用途 |
| --- | --- | --- |
| `/maxt_test_node/mission_status` | `std_msgs/String`（JSON） | 执行器发布任务状态和运行节点，快速编排 TUI 订阅；节点重命名或使用其他命名空间时需相应修改 `--task-topic` |
| `/mavros/state` | `mavros_msgs/State` | MavKit 订阅连接、解锁和飞行模式 |
| `/mavros/extended_state` | `mavros_msgs/ExtendedState` | MavKit 订阅接地状态 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | MavKit 订阅定位 |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | MavKit 订阅本地速度 |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | MavKit 发布位置设点 |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | MavKit 发布带掩码的原始设点 |
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | MavKit 调用解锁/上锁服务 |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | MavKit 调用飞行模式切换服务 |
| `/aruco_center/center` | `geometry_msgs/PointStamped` | 识别节点发布目标中心，x/y 为 u/v 像素坐标 |
| `/aruco_center/is_found` | `std_msgs/Bool` | 识别节点发布检测结果，FoundCheck 订阅 |
| `/aruco_center/debug_image` | `sensor_msgs/Image` | 识别节点发布标注图像 |

识别输出话题可以通过 YAML 更改；输入图像、相机话题及其配置见前面的使用说明。

#### Action 接口

Action 通过 actionlib 提供目标、结果和反馈，不是普通 ROS service。修改 `.action` 后，客户端和服务端都需要重新生成并加载消息。

| Action 名称 | 定义与角色 | Goal | Result | Feedback |
| --- | --- | --- | --- | --- |
| `/guidance` | [Guidance.action](action/Guidance.action)；Guidance 为客户端，识别或模拟反馈节点为服务端 | `string target_id` | `string message` | `bool target_visible`、`float32 error_u/error_v` |
| `/ugv_mover` | [ugv_mover.action](action/ugv_mover.action)；UgvCowork 或小车控制脚本为客户端，小车仿真为服务端 | `float32 target_x/target_y` | `bool succeed` | `uint8 state`：WAITING=0、STARTING=1、MOVING=2、REACHED=3 |

视觉误差单位必须与控制增益、容差匹配，现有 ArUco 服务使用像素误差。小车目标是绝对 XY，单位为米，Action 定义约定为 MAVROS 本地 ENU；服务端不负责坐标转换。`UgvCowork` 可通过 `action_name` 更改小车服务名，Guidance 当前使用 `/guidance`。

#### 行为树端口

端口定义以各节点 `providedPorts()` 为准。下表列出当前接口；未标注默认值的输入需要调用方提供，任务 XML 可以覆盖默认值。

| 注册名 | 输入端口（括号内为默认值） | 输出端口 |
| --- | --- | --- |
| `ConnectCheck` | `map_x/map_y/fact_x/fact_y(0)` | 无 |
| `CapturePose` | `height_offset(0)` | `x`、`y`、`z` |
| `Takeoff` | `target_alt(0.5)` | 无 |
| `Nav` | `x/y/z`、`tolerance(0.1)`、`timeout(60)` | 无 |
| `WaitStep` | `wait_duration(1)` | 无 |
| `Hover` | `hover_duration(1)` | 无 |
| `Round` | `center_x/center_y/center_z`、`radius(0)`、`angular_speed(30)`、`total_angle(360)`、`tolerance(0.3)`、`timeout(60)`、`ramp_time(2)` | 无 |
| `TouchDown` | 无 | 无 |
| `Guidance` | `target_id`、`timeout(30)`、`tracking_duration(0)` | `message` |
| `RegionTraverse` | `reg_point_1_x/y` 至 `reg_point_4_x/y`、`coverage_step` | `waypoints`、`waypoint_count` |
| `GetRegionWaypoint` | `waypoints`、`index` | `x`、`y`、`next_index` |
| `FoundCheck` | `topic(/aruco_center/is_found)` | 无 |
| `UgvCowork` | `action_name(/ugv_mover)`、`timeout(60)`、`follow_drone(false)`、`update_period(0.2)`、`target_tolerance(0.2)`、`settle_time(1)` | `message` |

以下参数与 Nav 参数统一存放在 `config/nav.yaml`，由所有行为树任务共用，不再作为 BT 端口。统一配置采用连接超时 60s、起飞超时 60s、上升速度 0.5m/s、下视相机开启、视觉比例增益 0.006；切换任务不会切换这些参数。未加载某个键时使用下表中的代码默认值。

| 私有参数前缀 | 参数及代码默认值 |
| --- | --- |
| `~connect_check/` | `timeout: 60.0` |
| `~capture_pose/` | `timeout: 10.0` |
| `~takeoff/` | `ascent_speed: 0.5`、`timeout: 30.0` |
| `~touch_down/` | `descent_speed: -0.5`、`timeout: 60.0`、`ground_timeout: 3.0`、`height_tol: 0.3`、`vel_tol: 0.2` |
| `~guidance/` | `tolerance: 5.0`、`downward_camera: false`、`kp: 0.01` |
| `~found_check/` | `timeout: 0.5` |

例如，修改 `config/nav.yaml` 的 `touch_down.descent_speed` 即可调整下降速度。`mission.launch` 在 `/maxt_test_node/` 下加载配置，因此实际参数名为 `/maxt_test_node/touch_down/descent_speed`。动作节点在每次启动时读取参数，FoundCheck 每次 tick 读取消息有效期；修改 YAML 后需重新加载参数才生效。

自定义行为树也使用这份统一配置，无需提供同名 YAML。更换无人机配置时，可用 `nav_config:=/absolute/path/params.yaml` 指定另一份完整配置：

```bash
roslaunch maxt_pkg mission.launch bt_xml_path:=/path/custom.xml nav_config:=/path/params.yaml
```

直接 `rosrun` 时，需要先通过 `rosparam load /path/params.yaml /maxt_test_node` 加载到正确的私有命名空间。快速任务脚本只生成 XML，起降和视觉控制参数在 `config/nav.yaml` 中调整。

时间通常以秒、距离以米、速度以 m/s 表示；Round 的角速度和总角度分别使用 °/s、°，MavKit 的 yaw 使用弧度。Guidance 容差沿用反馈误差单位。不同节点的超时可能使用 ROS 时间或墙钟时间，例如 FoundCheck、CapturePose 和 UgvCowork 使用墙钟，Hover 使用 ROS 时间；新增节点应明确选择时钟，并考虑仿真暂停时的行为。

黑板通过 `{键名}` 连接输入输出。`RegionWaypoints` 定义为 `std::vector<std::array<double, 2>>`，适合在节点间以强类型黑板值传递，不直接写成普通 XML 字符串。

### 类介绍

| 类或类型 | 职责与扩展位置 |
| --- | --- |
| `MaxtCore` | 持有 MavKit、BT 工厂、树、日志器和 AsyncSpinner；新增 XML 节点时修改 `registerNodes()` |
| `MavKit` | 管理 MAVROS 通信、状态快照、设点缓存和定时发布；飞行节点通过其接口控制无人机 |
| `MavActionNode` | 继承 `BT::StatefulActionNode`，保存注入的 `MavKit&`，作为飞行动作公共基类 |
| `FoundCheckNode` | 继承 `BT::ConditionNode`，根据检测值与消息新鲜度返回 SUCCESS/FAILURE |
| `RegionTraverseNode`、`GetRegionWaypointNode` | 同步规划和航点读取，不直接控制飞行 |
| `GuidanceNode` | 视觉 Action 客户端，根据反馈计算水平速度，管理等待、跟踪和完成状态 |
| `UgvCoworkNode` | 小车 Action 客户端，支持单目标及持续跟随无人机模式 |
| `QuinticCurve`、`CurveState` | 生成、采样三轴五次曲线，并保存参考位置/速度/加速度 |
| `PIDController` | 提供增益设置、积分/输出限幅、状态复位及控制量计算 |
| `CalibrateTools` | 根据参考方向计算 yaw 偏移，对 XY 位置、速度、加速度进行旋转 |

运行入口先调用 `MaxtCore::init()`，创建 MAVROS 通信、启动双线程 AsyncSpinner、注册节点并加载 XML，再由 `run()` 按 `bt_tick_rate` tick 根节点，直到 SUCCESS、FAILURE 或 ROS 关闭。MavKit 定时器按 `heartbeat_rate` 发布设点，因此 tick 频率与设点频率分别配置。

BT 回调和 ROS 回调可能并发执行，共享状态需要同步；例如 FoundCheck 用互斥锁保护检测值和接收时间。动作的每次 tick 应尽快返回，通过 RUNNING 保持执行，避免在生命周期函数中用长循环或 sleep 等待外部结果。

#### 函数介绍

以下是扩展飞行动作时最常用的接口，完整声明见 [maxt_mav.hpp](include/maxt_pkg/maxt_mav.hpp) 和 [maxt_nodes.hpp](include/maxt_pkg/maxt_nodes.hpp)。

| 函数 | 用途与调用约定 |
| --- | --- |
| `MaxtCore::init()` / `run()` | 初始化系统并运行行为树；XML 创建失败时 init 返回 false |
| `providedPorts()` | 声明 XML 输入输出的名称、类型和默认值 |
| `onStart()` | 动作从 IDLE 开始时读取参数、记录起始状态、发起请求；未完成时返回 RUNNING |
| `onRunning()` | 后续 tick 中检查反馈、完成条件与超时，返回 RUNNING/SUCCESS/FAILURE |
| `onHalted()` | 被父节点中断时取消本动作请求、释放控制；正常终止路径也要自行清理 |
| `getInput()` / `setOutput()` | 读取端口输入、将结果写到端口映射的黑板键 |
| `MavKit::requestArm(bool)` / `requestMode(string)` | 发起解锁或模式请求；实际状态应通过查询接口确认 |
| `isConnected()` / `isArmed()` / `isLand()` / `getMode()` | 获取连接、解锁、接地和模式状态 |
| `getCurrentPose()` / `getCurrentTwist()` | 获取状态快照，XY 已按标定偏移逆向旋转到工程世界坐标 |
| `get_current_yaw()` | 从接收的姿态计算 yaw，不额外应用标定偏移 |
| `setTargetPose(x,y,z)` | 更新位置目标缓存，随后选择 CONTROL；姿态默认单位四元数 |
| `setRawTarget(target)` | 更新 PositionTarget 缓存，随后选择 RAW_CTRL，并明确坐标帧及 type_mask |
| `setQuinticCurveTarget(position)` / `getCurveTime()` | 设置世界坐标目标、获取估算时长，随后选择 CURVE_CTRL |
| `setSetpointMode(mode)` | 切换发布方式，并处理减速或控制接管过渡 |
| `isReached(x,y,z,tolerance)` | 判断实测位置到目标的三维欧氏距离是否小于容差；各行为节点可能另有判定逻辑 |
| `updateHomePose()` / `getHomePose()` | 更新和查询 MavKit 内部 home 快照；与 CapturePose 写入的黑板键分别管理 |
| `mavCalibrate(mapx,mapy,factx,facty)` | 根据两组参考方向计算 XY 旋转，无平移、Z 或姿态补偿 |

`setTargetPose()`、`setRawTarget()` 更新缓存，实际消息由 MavKit 定时器发布。各模式语义如下：

| SetpointMode | 发布行为 |
| --- | --- |
| `STANDBY` | 不发布设点 |
| `HEARTBEAT` | 发布实时收到的当前位置；并非固定保存某次位置的悬停点 |
| `CONTROL` | 发布位置目标，应用工程坐标到定位坐标的 XY 旋转 |
| `RAW_CTRL` | 发布原始设点，按 type_mask 指定启用字段 |
| `CURVE_CTRL` | 采样五次曲线并发布参考位置、速度、加速度 |
| `SLOW_DOWN` | 生成并采样速度过渡曲线，完成后进入 HEARTBEAT 或待接管模式 |

需要固定位置悬停时，可参考 Hover：记录一次位姿，再持续使用该位置目标。Nav 中断时请求 SLOW_DOWN；减速期间的 HEARTBEAT 请求不会直接截断减速，新的导航目标可以继承当前参考状态重规划。`CurveState` 是控制参考，不是实测飞行状态，参考速度归零不等于飞机已完全停止。

坐标转换由 `CalibrateTools` 显式完成，不通过 TF 自动推导。`PositionTarget` 的字段、掩码、速度轴和坐标帧需要一致，尤其不能仅凭消息中的 `FRAME_LOCAL_NED` 常量就把工程中的所有坐标当作 NED。

### 新增行为树节点

1. 根据职责选择基类：飞行动作继承 `MavActionNode`；短时计算参考 `RegionTraverseNode` 或 `GetRegionWaypointNode`；状态判断参考 `FoundCheckNode`。
2. 创建头文件和源文件，声明 `providedPorts()`。持续动作实现 `onStart()`、`onRunning()`、`onHalted()`，同时处理成功、失败、超时和抢占时的清理。Action 客户端只取消本节点持有的目标，避免影响其他客户端。
3. 将源文件加入 `CMakeLists.txt` 的 `add_library(${PROJECT_NAME} ...)`。新增消息、库或 ROS 包依赖时同步更新 `package.xml` 和 CMake。
4. 在 `src/maxt_core.cpp` 包含头文件并注册。需要 MavKit 的节点使用 builder 注入已有实例，例如新增的 `InspectNode`：

   ```cpp
   factory_.registerBuilder<InspectNode>("Inspect",
       [this](const std::string& name, const BT::NodeConfiguration& config) {
           return std::make_unique<InspectNode>(name, config, *mav_);
       });
   ```

   这是待实现节点的注册示例，当前仓库没有 `InspectNode`。不需要额外构造参数的同步节点可使用 `factory_.registerNodeType<MyNode>("MyNode")`。

5. 在 `config/mission/` 添加 XML，通过注册名引用节点。新增 launch 时复用 `mission.launch` 加载配置，避免遗漏私有导航参数。
6. 编译后验证节点可创建、端口可解析，并覆盖动作完成、超时和被 halt 的行为；涉及真实控制时继续进行仿真验证。

新增 Python 可执行节点时使用 Python 3 shebang，赋予执行权限，并加入 `catkin_install_python(PROGRAMS ...)`。当前相机和监视脚本未全部列入该安装项，源码工作空间可运行不代表安装空间也包含相同入口。

### 编写任务与扩展接口

仅调整任务顺序时，修改 XML 即可，无需重新编译 C++。下面展示记录位置与使用黑板变量的方式，这是编排片段，不是完整飞行任务：

```xml
<root main_tree_to_execute="CaptureExample">
    <BehaviorTree ID="CaptureExample">
        <Sequence>
            <ConnectCheck />
            <CapturePose height_offset="4.0"
                         x="{home_x}" y="{home_y}" z="{return_z}" />
        </Sequence>
    </BehaviorTree>
</root>
```

需要在导航中持续检查目标时，参考 `region_traverse_test.xml` 中的 `ReactiveFallback`，检测成功会 halt 搜索分支。普通 Sequence 子节点失败后不会执行其后的降落节点；需要返航、降落等失败处理时，应像协同任务一样显式编排。使用 ForceSuccess 包裹动作会改变上层看到的结果，应通过 `message` 或业务结果黑板键保留真实执行原因。

修改视觉反馈协议时，保持 `.action`、感知服务端和 Guidance 客户端一致；更换小车服务端时，保留目标抢占、取消、反馈和最终 `succeed` 的约定。新增 YAML 参数需要明确加载命名空间，例如任务的 `nav/` 位于 `/maxt_test_node/nav/`，识别配置位于 `/aruco_center/`；在 launch 顶层加载同名参数不会自动成为节点私有参数。

### 开发验证

文档和配置修改先检查路径、节点注册名与端口是否一致。C++、Action 或依赖修改后，在工作空间中编译并重新加载环境：

```bash
cd ~/catkin_ws
catkin_make --pkg maxt_pkg
source devel/setup.bash
```

需要运行已有协同流程测试时，显式启用测试目标后执行：

```bash
cd ~/catkin_ws
catkin_make --pkg maxt_pkg -DCATKIN_ENABLE_TESTING=ON
source devel/setup.bash
rosrun maxt_pkg ugv_cowork_tree_test \
  "$(rospack find maxt_pkg)/config/mission/ugv_cowork_test.xml"
```

该测试覆盖任务分支与抢占流程，不能代替真实节点的控制验证。飞行相关修改应在仿真中检查实测轨迹、视觉反馈、超时和中断后的设点行为；可用 `rostopic hz` 检查发布频率，用 `rostopic echo` 查看状态与业务结果。只用行为树最终 SUCCESS 无法区分导航末端强制成功、跟踪失败后继续降落等情况。

本次仅补充开发文档，未执行上述编译、流程测试或仿真命令。
