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
   - 仅查看靶标场景：运行 `competition_world.launch`，不包含无人机。
   - 无人机仿真：先在外部完成 PX4 SITL 配置和构建，确保 `rospack find px4`、`rospack find mavlink_sitl_gazebo` 可解析，然后运行 `sim.launch`。
   - 实机任务：先启动 MAVROS 和定位系统，再启动任务入口。任务 XML 会实际请求解锁、OFFBOARD 和飞行，执行前需核对坐标、高度及运动范围。

任务入口使用 `sim` 和 `run_uav` 选择运行环境（默认 `sim:=true run_uav:=false`）：

| 参数 | 行为 |
| --- | --- |
| `sim:=true` | 启动任务和完整仿真（PX4、Gazebo、MAVROS）；忽略 `run_uav` |
| `sim:=false run_uav:=false` | 仅启动任务，复用已经启动的仿真或 MAVROS |
| `sim:=false run_uav:=true` | 启动任务和实机 MAVROS，连接 `/dev/ttyACM0:57600` |

`mission.launch` 支持上述仿真与实机参数。主任务入口 `sim_geo_treverse.launch` 默认启动仿真，`sim:=false` 可复用已有环境。模拟音频通过 `run_fake_audio:=true` 单独启用。

可选组件 `run_fast_lio` 使用 `tutorial_navigation/launch/mid360.launch`，`run_camera` 使用 `tutorial_vision/launch/simple_camera_driver.launch`；这两个外部包不随本项目提供，开关默认均为 `false`。

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
| `nav` | `x`、`y`、`z`、`timeout` |
| `hover` | `hover_duration` |
| `waitstep` | `wait_duration` |
| `round` | `center_x/y/z`、`radius`、`angular_speed`、`total_angle`、`tolerance`、`timeout`、`ramp_time` |
| `guidance` | `target_id`、`timeout`、`tracking_duration`（0 表示仅对齐） |

`Takeoff` 的上升速度及超时、`ConnectCheck` 的超时、`TouchDown` 的全部控制参数，以及 `Guidance` 的 `tolerance/downward_camera/kp` 从 `config/nav.yaml` 对应模块读取，不再作为这些节点的 XML 属性生成。`Nav`、`Round` 和 `Guidance` 的任务 `timeout` 仍是 BT 输入端口。`Guidance.message` 为输出端口，当前生成器不要求填写。运行 `python3 -B -m unittest discover -s test -p 'test_fast_mission.py'` 可检查生成器定义、端口默认值和 XML 是否与节点源码一致。

输入 `0` 后覆盖包内 `config/mission/fast_mission.xml`，树名固定为 `FastMission`。脚本始终保留首项 `<ConnectCheck />` 和末项 `<TouchDown />`，中间按添加顺序生成 XML。脚本只生成文件；执行任务使用：

```bash
roslaunch maxt_pkg mission.launch bt_xml_path:="$(rospack find maxt_pkg)/config/mission/fast_mission.xml"
```

通用入口自动加载 `config/nav.yaml`；快速编排器生成的 XML 可通过 `bt_xml_path` 显式执行。

该顺序树中，前面的任务失败会使整棵树失败，后续降落节点也不会执行。

下面的命令均在已加载工作空间的终端执行。`rosrun` 不会自动加载 launch 中的 YAML，也不会启动依赖节点。

#### 任务执行：maxt_test_node

运行自定义行为树时，需要指定 XML，并将导航配置加载至该节点的私有命名空间：

```bash
rosparam load "$(rospack find maxt_pkg)/config/nav.yaml" /maxt_test_node
rosrun maxt_pkg maxt_test_node \
  _bt_xml_path:="$(rospack find maxt_pkg)/config/mission/traverse_guidance_mission.xml" \
  _bt_tick_rate:=20.0 _heartbeat_rate:=20.0
```

通常直接使用 `mission.launch` 更方便，它会完成参数加载。`Nav` 的速度、加速度及 PID 参数来自私有 `nav/` 参数；到达位置容差使用 `config/nav.yaml` 的 `reach_check/position_pose_tolerance`（默认 0.1 m），不再提供 XML `tolerance` 端口。

`Nav`、`NavAzimuth` 和 `NavGeo` 共用 `nav/hold_timeout`，在每次启动航行时读取，未配置时默认 3.0 秒。从曲线结束后首次进入末端对齐阶段开始计时（NavGeo 等待平移和航向曲线均结束）；超过该时长仍未达标则切回 HEARTBEAT 并强制返回 SUCCESS。任务总超时仍优先返回 FAILURE。该参数是 ROS 配置，不是 XML 端口。

`Nav`、`NavAzimuth` 和 `NavGeo` 在行为树中被 halt 中断时，会在请求减速之前记录当时的实测位置。新增可选输出端口 `breakpoint_x/y/z` 和 `breakpoint_valid`，将端口映射到黑板即可保存。三个节点输出均为相对内部 Home 的标定坐标（米），可直接交给 `Nav` 返回断点，包括从经纬度导航中断后返回原高度。输出仅在中断时更新，成功、失败和重新启动不会清除断点；尚未中断时输出未初始化，应先在黑板初始化 `bp_valid=false`。没有有效定位或坐标非有限数时，仅将有效标志设为 false，不覆盖旧坐标。调用方必须检查有效标志后再返回。

```xml
<!-- 放在可被高优先级分支中断的导航分支中 -->
<Nav x="10" y="0" z="4"
     breakpoint_x="{bp_x}" breakpoint_y="{bp_y}" breakpoint_z="{bp_z}"
     breakpoint_valid="{bp_valid}"/>
<!-- 中断任务结束后，在确认 bp_valid 为 true 的返回分支中执行 -->
<Nav x="{bp_x}" y="{bp_y}" z="{bp_z}"/>
```

断点保存在当前行为树黑板内，不跨进程重启保存；共用同一组黑板键时以最后一次中断为准，需要分别保留时请映射到不同键。记录后到返回前应保持内部 Home 和标定参数不变。此功能提供断点记录和返回目标，不自动改变行为树分支或恢复剩余航线。断点是触发中断时的位置，并非减速后的停止点；超时返回 FAILURE 不属于 halt 中断。

当前注册的行为树节点为：`ConnectCheck`、`CapturePose`、`Takeoff`、`Nav`、`NavGeo`、`BasePointAzimuthFix`、`NavAzimuth`、`WaitStep`、`Hover`、`Round`、`TouchDown`、`Guidance`、`TraversePathGenerator`、`GetRegionWaypoint`、`FoundCheck`、`UgvCowork`、`PlayAudio`。这些是 XML 节点，不是独立的 `rosrun` 可执行文件。`Land`、`XNavigation` 未注册。

方位角固定航行使用 `BasePointAzimuthFix` 和 `NavAzimuth`。前者在一次 tick 内调用 `GetRawPose()`，将当前原始位置写入黑板；后者以该基点为原点，按照 `config/nav.yaml` 中的 `nav/azimuth_angle` 投影位移。角度单位为度，0° 沿原始 X 正向，90° 沿原始 Y 正向；`nav_n` 沿该方向，`nav_t` 正向为左侧，`nav_a` 为相对基点的 Z 增量，距离单位为米。每个目标都相对于保存的基点，不累计上一段位移，也不设置机头朝向。

目标原始坐标为 `x = bp_x_raw + nav_n*cos(angle) - nav_t*sin(angle)`、`y = bp_y_raw + nav_n*sin(angle) + nav_t*cos(angle)`、`z = bp_z_raw + nav_a`。新节点复用五次曲线和 PID 工具，在 BT tick 中采样曲线并通过 `RAW_CTRL` 发布原始位置、速度、加速度，不应用 Home 偏移或标定旋转。曲线起点采用当前原始位置，起终点参考速度和加速度均为零；与已有 Nav 的连续参考重规划不同。速度、加速度、PID 配置仍使用 `nav/`，到达位置容差使用 `reach_check/position_pose_tolerance`。沿用 Nav 的 60 秒默认超时、末端按 `nav/hold_timeout`（默认 3 秒）强制成功及中断减速逻辑；尚未进行仿真飞行验证。

例如，在完成连接和起飞后记录基点，再沿配置方向移动 2 m 并保持基点高度：

```xml
<Sequence>
    <BasePointAzimuthFix bp_x_raw="{bp_x_raw}" bp_y_raw="{bp_y_raw}" bp_z_raw="{bp_z_raw}" />
    <NavAzimuth nav_n="2" nav_t="0" nav_a="0"
                   bp_x_raw="{bp_x_raw}" bp_y_raw="{bp_y_raw}" bp_z_raw="{bp_z_raw}"
                   timeout="60" />
</Sequence>
```


#### 语音播报

MavKit 创建时提交 `cn-welcome.wav`；`ConnectCheck` 确认飞控连接并完成标定、即将返回 SUCCESS 时提交一次 `cn-ready.wav`。重新执行连接检查并成功时会再次提交，单纯断开重连不会自动播报。`stateCb` 只更新状态，不执行任何播放操作；解锁和上锁提示由起飞和降落行为树节点确认对应状态后触发。

`Takeoff` 在等待解锁阶段确认已解锁时提交一次 `cn-armed.wav`；OFFBOARD 确认后、初始化爬升时提交 `cn-takeoff.wav`，进入爬升阶段后提交一次 `sample-6s.wav`。每次重新执行起飞节点会重置爬升播报触发。`TouchDown` 在已解锁、开始降落时提交 `cn-landing.wav`，进入下降阶段后提交一次 `cn-welcome-sua.wav`；在等待上锁阶段确认已上锁时提交 `cn-disarmed.wav`。每次重新执行降落节点会重置下降播报触发；节点启动时已经上锁则直接成功，不播报降落或上锁提示。未执行对应行为树节点时，外部解锁/上锁不会触发这两条提示。

所有事件复用 `AudioCaller` 调用 `/audio/play`，类型为本包生成的 `maxt_pkg/PlayAudio`，与相同定义及 MD5 的 `uav_audio/PlayAudio` 兼容。服务检查和调用在单个后台线程中执行；主任务不等待结果，析构也不等待正在进行的服务调用。服务不存在、服务端 busy、客户端正在调用或已有待发请求时均可能丢弃播报，不重试。相邻事件的提示可能被丢弃，例如解锁提示尚在播放时起飞提示不会排队；该功能不保证每条音频实际播出。

行为树可通过文件名触发自定义播报（不是文本转语音）：

```xml
<PlayAudio filename="cn-mission-start.wav" />
```

`PlayAudio` 每次执行只提交一次请求，立即返回 SUCCESS；音频服务缺失或拒绝不影响任务。缺少必填 `filename` 输入时返回 FAILURE。在循环或重新执行的树中，每次执行都会再次提交。


无需音频设备的联调方式（编译并加载工作空间后）：

```bash
rosrun maxt_pkg fake_audio.py _duration:=3.0
rosservice call /audio/play "filename: 'cn-ready.wav'"
```

模拟服务立即响应，每次默认模拟播放 3 秒；新的合法请求抢占当前播放并重新计时，不返回 busy，非法请求不会中断当前播放。不检查真实文件，也不发声。真实 `uav_audio` 服务仍采用忙时拒绝策略。`mission*.launch` 仅在 `run_fake_audio:=true` 时通过公共入口 `mission.launch` 启动 `fake_audio`；默认关闭，不受 `sim` 影响。真实播放需另行启动 `uav_audio` 节点，并在服务端包的 `audio/` 目录中放置对应 WAV。

#### 视觉识别：aruco_center.py

识别指定字典和 ID 列表的 ArUco，发布中心像素坐标、检测状态与标注图像；可选提供 `/guidance` Action 服务。

```bash
rosparam load "$(rospack find maxt_pkg)/config/aruco_center.yaml" /aruco_center
rosrun maxt_pkg aruco_center.py _run_guidance:=true
```

默认 YAML 输入为 `/camera/rgb/image_raw`，字典为 `DICT_4X4_50`，`marker_ids: [0, 1, 2, 3, 4, 5]` 对应 `models/targets/materials/textures/` 中的全部标记。多个 ID 同时可见时，中心和 `/aruco_center/content`（`String`）发布列表中最靠前的可见目标；无目标时内容为空。`/guidance` 可按 `target_id` 独立跟踪列表内任一目标，目标丢失时反馈不可见，不会切换到其他 ID。删除 `marker_ids` 参数后兼容原来的单目标 `marker_id` 配置。输出为 `/aruco_center/center`（`PointStamped`，x/y 为像素坐标）、`/aruco_center/is_found`（`Bool`）、`/aruco_center/debug_image`（`Image`）。标注图像仅在有订阅者时发布。使用 USB 相机时将 `_image_topic:=/camera/image_raw` 加到命令末尾。

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
roslaunch maxt_pkg sim.launch move_target:=false
```

另一个终端运行：

```bash
rosrun maxt_pkg moving_target.py _x:=-4.0 _y:=4.0 _amplitude:=2.0 _period:=25.13
```

脚本等待 `/gazebo/set_model_state`，按仿真时钟沿 Y 轴正弦运动。若 `/moving_target/enabled` 为 true，会拒绝启动以避免与插件竞争。该脚本读取自己的私有参数，不会自动使用 `config/sim_world.yaml`。

### launchs

#### 当前主任务：TraverseGuidanceMission

```bash
roslaunch maxt_pkg sim_geo_treverse.launch
# 复用已启动的仿真或飞控环境：
roslaunch maxt_pkg sim_geo_treverse.launch sim:=false
```

主任务加载 `config/mission/traverse_guidance_mission.xml`，依次执行连接检查、起飞、航向调整、经纬度导航、基点采集、区域遍历与目标引导、返航和降落。任务区域及经纬度在 XML 中设置，航向使用 `config/nav.yaml` 的 `nav/azimuth_angle`。

| 启动文件 | 用途 |
| --- | --- |
| `sim_geo_treverse.launch` | 主任务入口，默认启动仿真、识别和视觉引导 |
| `mission.launch` | 通用执行入口，需传入 `bt_xml_path`，支持实机和自定义配置 |
| `aruco_center.launch` | ArUco 识别，`run_guidance:=true` 开启视觉引导 |
| `sim.launch` | PX4 SITL、Gazebo 和 MAVROS |
| `competition_world.launch` | 比赛场景，可单独查看或由仿真加载 |
| `mavros.launch` | 实机 MAVROS 连接 |

实机任务可先单独启动识别，再使用通用入口：

```bash
roslaunch maxt_pkg aruco_center.launch run_guidance:=true
# 另一终端：
roslaunch maxt_pkg mission.launch bt_xml_path:="$(rospack find maxt_pkg)/config/mission/traverse_guidance_mission.xml" sim:=false run_uav:=true
```

其他任务的顶层 XML 和专用 launch 已清理；`config/mission/subtree/` 仅保留主任务依赖的 4 个子树。

### 常用配置

| 文件 | 当前关键值与用途 |
| --- | --- |
| [config/nav.yaml](config/nav.yaml) | `v_max=5 m/s`、`a_max=1 m/s²`；PID 为 `1.0/0.05/0.0`；减速时间 1s |
| [config/aruco_center.yaml](config/aruco_center.yaml) | 仿真相机输入、`DICT_4X4_50`、ID 0～5、检测及图像输出话题 |
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
│   ├── mavkit/maxt_mav.hpp          # MavKit、SetpointMode、CurveState、点获取辅助接口
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
| `PlayAudio` | `filename` | 无 |
| `Takeoff` | `target_alt(0.5)` | `home_x`、`home_y`、`home_z`、`return_z` |
| `NavGeo` | `latitude/longitude`、`timeout(60)` | `breakpoint_x/y/z`、`breakpoint_valid` |
| `Nav` | `x/y/z`、`timeout(60)` | `breakpoint_x/y/z`、`breakpoint_valid` |
| `BasePointAzimuthFix` | 无 | `bp_x_raw/bp_y_raw/bp_z_raw` |
| `NavAzimuth` | `nav_n/nav_t/nav_a`、`bp_x_raw/bp_y_raw/bp_z_raw`、`timeout(60)` | `breakpoint_x/y/z`、`breakpoint_valid` |
| `WaitStep` | `wait_duration(1)` | 无 |
| `Hover` | `hover_duration(1)` | 无 |
| `Round` | `center_x/center_y/center_z`、`radius(0)`、`angular_speed(30)`、`total_angle(360)`、`tolerance(0.3)`、`timeout(60)`、`ramp_time(2)` | 无 |
| `TouchDown` | 无 | 无 |
| `Guidance` | `target_id`、`timeout(30)`、`tracking_duration(0)` | `message` |
| `TraversePathGenerator` | `origin_x/y`、`length_x/y`、`coverage_step` | `waypoints`、`waypoint_count` |
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
| `TraversePathGeneratorNode`、`GetRegionWaypointNode` | 同步规划和航点读取，不直接控制飞行 |
| `GuidanceNode` | 视觉 Action 客户端，根据反馈计算水平速度，管理等待、跟踪和完成状态 |
| `UgvCoworkNode` | 小车 Action 客户端，支持单目标及持续跟随无人机模式 |
| `QuinticCurve`、`CurveState` | 生成、采样三轴五次曲线，并保存参考位置/速度/加速度 |
| `PIDController` | 提供增益设置、积分/输出限幅、状态复位及控制量计算 |
| `CalibrateTools` | 根据参考方向计算 yaw 偏移，对 XY 位置、速度、加速度进行旋转 |

运行入口先调用 `MaxtCore::init()`，创建 MAVROS 通信、启动双线程 AsyncSpinner、注册节点并加载 XML，再由 `run()` 按 `bt_tick_rate` tick 根节点，直到 SUCCESS、FAILURE 或 ROS 关闭。MavKit 定时器按 `heartbeat_rate` 发布设点，因此 tick 频率与设点频率分别配置。

BT 回调和 ROS 回调可能并发执行，共享状态需要同步；例如 FoundCheck 用互斥锁保护检测值和接收时间。动作的每次 tick 应尽快返回，通过 RUNNING 保持执行，避免在生命周期函数中用长循环或 sleep 等待外部结果。

#### 函数介绍

以下是扩展飞行动作时最常用的接口，完整声明见 [maxt_mav.hpp](include/maxt_pkg/mavkit/maxt_mav.hpp) 和 [maxt_nodes.hpp](include/maxt_pkg/maxt_nodes.hpp)。

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
| `GetRawPose()` | 获取原始 MAVROS 位姿，不做 Home 抵消或角度标定 |
| `RecordRawHomePose()` | 起飞前将完整原始位姿保存至 `raw_home_pose_`，独立于用于坐标偏移的 Home |
| `GetRawHomePose()` | 返回已记录的起飞前原始位姿快照，不做 Home 抵消或角度标定 |
| `GetHomeOffsetPose()` | 原始坐标减去 Home 的 XYZ，不做角度标定 |
| `GetHomeCalibratePose()` | 先抵消 Home 的 XYZ，再按标定偏移逆向旋转 XY |
| `GetCalibratePose()` | 仅逆向旋转 XY 到工程世界坐标，不抵消 Home |
| `getCurrentTwist()` | 获取速度快照，XY 已按标定偏移逆向旋转到工程世界坐标 |
| `get_current_yaw()` | 从接收的姿态计算 yaw，不额外应用标定偏移 |
| `setTargetPose(x,y,z)` | 更新位置目标缓存，随后选择 CONTROL；姿态默认单位四元数 |
| `setRawTarget(target)` | 更新 PositionTarget 缓存，随后选择 RAW_CTRL，并明确坐标帧及 type_mask |
| `setQuinticCurveTarget(position)` / `getCurveTime()` | 设置世界坐标目标、获取估算时长，随后选择 CURVE_CTRL |
| `setSetpointMode(mode)` | 切换发布方式，并处理减速或控制接管过渡 |
| `ReachAltitudeCheck(altitude_raw)` | 原始高度误差和垂直速度绝对值均在配置容差内时返回 true |
| `ReachPoseCheck(pose_raw)` | 原始三维位置距离和线速度模长均在配置容差内时返回 true |
| `isReached(x,y,z,tolerance)` | 判断实测位置到目标的三维欧氏距离是否小于容差；各行为节点可能另有判定逻辑 |
| `updateHomePose()` / `getHomePose()` | 更新和查询 MavKit 内部 home 快照；与 CapturePose 写入的黑板键分别管理 |
| `mavCalibrate(mapx,mapy,factx,facty)` | 根据两组参考方向计算 XY 旋转，无平移、Z 或姿态补偿 |

点获取逻辑集中在 `include/maxt_pkg/mavkit/maxt_mav.hpp` 和 `src/mavkit/get_pose.cpp`。
四种查询通过 `maxt::MavKit` 调用，均保留原始消息的 header 和 orientation；
角度标定只作用于位置 XY，不改变 Z 或姿态。Home 使用 `updateHomePose()`
记录的原始 MAVROS 快照，未记录时默认为零；它与 CapturePose 的黑板输出分别管理。
现有导航节点使用 `GetCalibratePose()`，目标设点的坐标语义保持不变。

到达检查由 `MavKit` 直接管理容差和判定逻辑，实现位于 `src/mavkit/reach_check.cpp`，
可通过 `include/maxt_pkg/mavkit/maxt_mav.hpp` 引入接口。
输入目标须使用 MAVROS 原始局部坐标（高度是该坐标系的 Z，不是相对 Home 高度），
检查不做 Home 抵消或角度标定，也不检查目标姿态。速度目标为零，容差边界包含等号。
`config/nav.yaml` 的 `reach_check` 配置由 MavKit 在 `mavInit()` 时通过传入的 NodeHandle 读取，
任务启动时位于节点私有命名空间：`vertical_alt_tolerance=0.1 m`、
`vertical_vel_tolerance=0.2 m/s`、`position_pose_tolerance=0.1 m`、
`position_vel_tolerance=0.2 m/s`，未配置时默认值相同。
调用方应在定位与速度数据就绪后调用；每次检查使用持锁读取的原始状态，不要求持续停留时间。
现有节点及 `isReached()` 的判断逻辑保持原样。

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

1. 根据职责选择基类：飞行动作继承 `MavActionNode`；短时计算参考 `TraversePathGeneratorNode` 或 `GetRegionWaypointNode`；状态判断参考 `FoundCheckNode`。
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

需要在导航中持续检查目标时，参考 `subtree/NavgeoDetectTree.xml` 中的检测与导航编排，检测成功会 halt 当前航段并在处理断点后继续巡航。普通 Sequence 子节点失败后不会执行其后的降落节点；需要返航、降落等失败处理时，应像协同任务一样显式编排。使用 ForceSuccess 包裹动作会改变上层看到的结果，应通过 `message` 或业务结果黑板键保留真实执行原因。

修改视觉反馈协议时，保持 `.action`、感知服务端和 Guidance 客户端一致；更换小车服务端时，保留目标抢占、取消、反馈和最终 `succeed` 的约定。新增 YAML 参数需要明确加载命名空间，例如任务的 `nav/` 位于 `/maxt_test_node/nav/`，识别配置位于 `/aruco_center/`；在 launch 顶层加载同名参数不会自动成为节点私有参数。

### 开发验证

文档和配置修改先检查路径、节点注册名与端口是否一致。C++、Action 或依赖修改后，在工作空间中编译并重新加载环境：

```bash
cd ~/catkin_ws
catkin_make --pkg maxt_pkg
source devel/setup.bash
```

飞行相关修改应在仿真中检查轨迹、视觉反馈、超时和中断后的设点行为。旧任务 XML 对应的流程测试已从 CMake 注册中移除。

标定工具单元测试无需启动 ROS master，覆盖默认变换、标定更新、XY 旋转方向、
消息字段转换及保留字段。`untransRaw(PositionTarget&)` 当前只逆变换位置和速度，
不会逆变换加速度；测试按此接口行为验证。

```bash
cd ~/catkin_ws
catkin_make --pkg maxt_pkg -DCATKIN_ENABLE_TESTING=ON --make-args test_calibrate_tools
source devel/setup.bash
rosrun maxt_pkg test_calibrate_tools
```

### 遍历引导子树

`TraverseGuidanceMission` 在主任务中规划区域航点，并调用以下子树：

- `traverse_guidance_mission.xml`：规划区域航点，先遍历并记录目标，再逐个返回记录点伺服。
- `TraverseRecoreTree.xml`：飞行期间持续检测，按 ID 累计目标观察位置并取平均。
- `RollbackTargetTree.xml`：遍历结束后按距离从近到远返回记录点并伺服，跳过已完成目标。

`TargetCheck` 对同一 ID 的有效检测消息逐条累计，将无人机原始本地位置 x/y/z 的算术平均写入 `target_local_poses`，累计次数写入 `target_local_pose_sample_counts`。同一消息不会因重复 tick 被重复计数，跨航段继续累计；不同 ID 独立统计。姿态和时间戳沿用最新有效样本，`target_pose` 输出与黑板中的平均位姿一致。过期、未定位、非有限位置及已完成目标的样本不参与平均；`record="false"` 仍只检查可见性。若要重置记录，应同时清空位姿和累计次数。这是观察位置的平均，不是目标坐标估计，单次观测仍只能保留该位置。

`ListRecordedTargets` 默认启用 `sort_by_distance="true"`：以执行节点时无人机的原始本地位置为固定起点，按到各记录点的三维直线距离升序输出 ID，等距时按 ID 排列。一次生成整个列表，不在每次伺服后重新排序，也不保证全程最短路径。设置为 `false` 可恢复按 ID 排序。距离排序需要有效定位及同一坐标系的有效记录，否则节点失败；没有待处理目标时直接成功。

区域原点偏移、边长和航带间距在主任务 XML 中配置，方向使用 `nav/azimuth_angle`。子树不负责起飞、返航或降落，这些步骤由主任务编排。

`Takeoff` 持续等待外部解锁，不主动发送解锁请求，不设置等待解锁超时。检测到解锁后重新计时，使用 `takeoff.timeout` 限制 OFFBOARD 切换和爬升阶段。

`Takeoff` 在开始爬升前等待有效定位，将标定坐标系中的起飞位置输出到 `home_x/home_y/home_z`，并将起飞 Z 加 `target_alt` 输出到 `return_z`。输出端口按需映射到黑板，每次执行重新记录，爬升过程中不覆盖。例如 `<Takeoff target_alt="4.0" home_x="{home_x}" home_y="{home_y}" home_z="{home_z}" return_z="{return_z}" />`；随后用 `<Nav x="{home_x}" y="{home_y}" z="{return_z}" />` 返回起飞点上方，再执行 `<TouchDown />`。

初始化爬升时先调用 `RecordRawHomePose()` 记录未经处理的起飞位姿，再读取 `GetRawPose()`，保持该原始局部坐标的 XY，以 `takeoff.ascent_speed` 恒定垂直速度通过 `setRawTarget()` 和 `RAW_CTRL` 上升。目标原始高度为该次读取的 Z 加 `target_alt`，通过 `ReachAltitudeCheck()` 判定到达，随后切回 HEARTBEAT。

`TouchDown` 使用 `GetRawPose()` 的 XY 作为下降保持位置，通过 `RAW_CTRL` 按配置速度下降。接地高度基准取自起飞前 `RecordRawHomePose()` 保存的位姿，即 `GetRawHomePose()` 的 Z；当前高度同样读取原始位姿，不做 Home 偏移或角度标定。原有高度、垂直速度、接地状态联合确认及持续确认时间、超时和上锁流程保持不变。

### 经纬度导航

`NavGeo` 使用 WGS-84 经纬度绝对目标（度），只接收 `latitude`、`longitude` 和可选的 `timeout`，不接收 `altitude`。航行期间固定启动时的高度，由飞控高度控制器保持，不规划升降曲线。例如：

```xml
<NavGeo latitude="31.230400" longitude="121.473700" timeout="60"/>
```

`NavGeo` 启动时读取 ROS 参数 `nav/azimuth_angle`（度，默认 0），平移与航向五次曲线同时启动，航向沿最短转向调整到该角度。角度使用原始 ENU 坐标：+X 为 0°，朝 +Y 为正，与 `ReHeading` 一致。航向曲线复用 `re_heading/yaw_rate_max`（默认 0.5 rad/s）和 `re_heading/yaw_acc_max`（默认 0.5 rad/s²）；航向曲线结束后，使用 `nav/pid_*` 输出限幅的航向角速度进行末端 PID 对齐，测量角连续展开以避免跨越 ±180° 时微分跳变。正常到达需要位置、高度、速度及航向误差同时达标，航向容差复用 `re_heading/tolerance_deg`（默认 2°）。成功时将目标航向保存为默认航向；总超时、定位失效或中断时保存当前实测航向。

上述坐标仅为端口示例，执行任务前应替换为实际航点。节点运行前需要有效的 `/mavros/global_position/global`（`sensor_msgs/NavSatFix`）、`/mavros/global_position/rel_alt`（`std_msgs/Float64`）及 `/mavros/local_position/pose`。不依赖海拔或 NavSatFix 的椭球高。原 `LaLoAlNav` 节点已更名为 `NavGeo`，旧 XML 需更换节点名称并移除 `altitude` 属性。

启动时记录 local Z 用于高度保持误差的到达检查，记录飞控相对 Home 高度作为固定的全球设点高度。MavKit 的 `GetGlobalPose()` 返回经纬度及飞控相对 Home 高度，`hasGlobalPose()` 检查定位、相对高度及 local 位姿是否就绪。`SetGlobalRawTarget()` 设置全球 raw 设点，`GLOBAL_RAW_CTRL` 由心跳定时器向 `/mavros/setpoint_raw/global` 持续发布。节点使用 `FRAME_GLOBAL_REL_ALT`，显式发布航向曲线/PID 设点（`SetGlobalRawTarget(target, false)`），不受已有默认航向覆盖，不需要叠加应用 Home Z。

NavGeo 启动时计算预计曲线时长 T，通过 `config/nav.yaml` 的 `nav/lalo_max_duration` 设置安全上限（默认 60.0 秒，须为有限正数）。若 T 超过上限，记录错误并返回 FAILURE，在发送全球设点、切换 GLOBAL_RAW_CTRL 前拒绝启动。T 等于上限时允许启动。此安全门替代原 XY 偏移限制；T 仅指平移曲线时长，不包含航向曲线时长及 PID 末端对齐时间，与 XML 的运行总超时 `timeout` 独立。

`GlobalNavCurve` 仅对水平测地距离生成五次曲线，再转换为经纬度及 ENU 水平速度、加速度；高度始终固定为起点高度，Z 速度和 Z 加速度均为零。时长估算及 `nav/v_max`、`nav/a_max` 与 Nav 一致。BT tick 采样曲线，定时器发布最近一次设点。曲线结束后仅对东、北方向做 PID 速度修正，复用 `nav/pid_kp/pid_ki/pid_kd`，各轴输出限幅 0.5 m/s、积分限幅 0.3，不做高度 PID 对齐。飞控负责位置控制及高度保持，前馈支持情况需结合飞控验证。

`ReachGlobalCheck(target, altitude_raw)` 同时检查水平距离、相对启动 local Z 的高度误差和实测速度。对应配置为 `reach_check/global_position_tolerance`（代码默认 1.0 m）、`reach_check/global_altitude_tolerance`（默认 0.5 m）及 `reach_check/position_vel_tolerance`（默认 0.2 m/s）。水平距离使用 GeographicLib 的 WGS-84 椭球测地距离，容差单位为米。

末端对齐共用 `nav/hold_timeout`（默认 3 秒）：平移和航向曲线均结束后，从首次进入联合末端对齐阶段计时（较早结束的轴提前进行 PID 对齐），达标立即返回 SUCCESS，否则超过配置时长后记录警告、切回 HEARTBEAT 并强制成功，因此 SUCCESS 不保证误差达标。任务总超时优先返回 FAILURE 并切回 HEARTBEAT；定位或相对高度无效时返回 FAILURE 并请求 SLOW_DOWN，中断同样请求 SLOW_DOWN。曲线起终点参考速度均为零。尚未进行仿真飞行验证。

导航中断后的 Guidance 使用 FoundCheck 成功时输出的 `target_id`，通过子树黑板传递并锁定，不再固定跟踪 ID 0。返回断点期间新出现的 ID 不会改写该目标。Guidance 启动日志显示目标 ID、跟踪时长和总超时；目标始终不可见时，达到总超时返回 `TIMEOUT`。

目标黑名单 `processed_qr` 由 Guidance 在成功收尾时更新（再次返回断点前）。FoundCheck 只检查历史并输出目标 ID；仅发现、伺服失败、超时或中止均不会加入黑名单。`record="false"` 仍只检查可见性。

导航中断处理在 Guidance 成功后执行 `SendGeoPose`，将新的 GPS 定位发布到 `QRCode_pose/Global_pose`，发送成功后再返回断点继续原航段。等待有效定位或接收端超过 10 秒时返回 FAILURE。
