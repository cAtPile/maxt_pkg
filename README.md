# Mavros eXtra function based on Behavior Tree

`maxt_pkg` 使用 BehaviorTree.CPP v3 编排 MAVROS 无人机基础飞行任务。

MaxtCore 负责初始化、注册行为树节点和执行 XML；MavKit 封装状态订阅、坐标标定、设点发布以及解锁和模式服务。任务节点使用已有非阻塞生命周期。

当前注册节点：`ConnectCheck`、`Takeoff`、`GoTo`、`Nav`、`WaitStep`、`Hover`、`Round`、`TouchDown`、`Guidance`、`RegionTraverse`、`GetRegionWaypoint`、`FoundCheck`、`UgvCowork`、`CapturePose`。`Land`、`XNavigation` 仅有历史头文件声明，未注册，不能用于任务 XML。

比赛专用识别、圆环检测和投放功能已移除，接口变更和建议见 [清理交付文档](docs/done/craicRM_delivery.md)。

主要目录：

- `include/maxt_pkg/`、`src/`：核心、MAVROS 抽象层和行为树节点。
- `nodes/maxt_test_node.cpp`：任务执行入口。
- `config/mission/`：基础、航行和圆周运动测试配置。
- `launch/mission.launch`：通用任务入口。
- `launch/basic_test.launch`、`navigation_test.launch`、`round_test.launch`：三个测试入口。
- `script/simple_camera_driver.py`：通用相机脚本。
- `script/tools/`：速度和轨迹监视工具。

依赖：ROS、catkin、MAVROS、BehaviorTree.CPP v3、geometry_msgs、std_msgs、tf2、Boost。通用相机脚本另外需要 rospy、sensor_msgs、cv_bridge 和 OpenCV。

在 catkin 工作空间编译并加载环境后，先准备 MAVROS 和定位系统，再执行：

```bash
roslaunch maxt_pkg basic_test.launch
```

该任务会实际解锁、进入 OFFBOARD、起飞至 1.0m、悬停 3s，然后通过 TouchDown 降落。配置复用既有飞行行为，高度和坐标语义以现有 MavKit 和节点实现为准。

新增测试套件均包含连接检查、起飞、测试动作、返回原点和降落（基础测试在原地降落）：

| 配置 | 启动命令 | 测试动作 |
| --- | --- | --- |
| `basic_test.xml` | `roslaunch maxt_pkg basic_test.launch` | 起飞 1m，悬停 3s |
| `navigation_test.xml` | `roslaunch maxt_pkg navigation_test.launch` | Nav 依次航行至 (0,0,1)、(2,0,1)、(2,2,1)、(0,0,1)，各点悬停 |
| `round_test.xml` | `roslaunch maxt_pkg round_test.launch` | 航行至 (0,0,1)，绕 (1,0,1) 顺时针一圈，半径 1m、角速度 20°/s，返回 (0,0,1) |

坐标单位为米，使用 MavKit 当前坐标系中的绝对坐标；这两个运动测试假设起飞点 XY 为 (0,0)，目标高度 z 为 1.0。若定位原点或地面高度不同，请先修改 XML 中的航点、圆心和高度。ConnectCheck 默认不启用坐标标定。导航限速为 0.8m/s，限加速度为 0.5m/s²；圆周加减速时间为 2s，超时为 45s。三个入口均支持相同的外部组件和频率开关。

Nav 现有实现会在末端等待超过 3s 后强制返回 SUCCESS，因此行为树成功日志不能单独证明航点精度达标；请结合实际轨迹和误差检查结果。

如果本机已有 `tutorial_basic` 和 `tutorial_navigation`，可由启动文件一并启动 MAVROS 和 FAST-LIO：

```bash
roslaunch maxt_pkg basic_test.launch run_mavros:=true run_fast_lio:=true
```

自定义任务使用：

```bash
roslaunch maxt_pkg mission.launch bt_xml_path:=/absolute/path/to/task.xml
```

`mission.launch` 支持 `bt_tick_rate`、`heartbeat_rate`（默认均为 20Hz），以及默认关闭的 `run_mavros`、`run_fast_lio`、`run_camera`。三个组件开关分别依赖外部包 `tutorial_basic`、`tutorial_navigation`、`tutorial_vision`。

本次按指南要求未执行编译或运行测试，测试入口供人工检验。

## Gazebo 视觉仿真

新增 ROS Noetic / Gazebo Classic 11 场景，包含地面、三个方柱障碍物、沿 Y 轴往返的图片板；识别图像来自无人机机身下视相机：

```bash
roslaunch maxt_pkg vision_test.launch
# 查看机身相机原始彩色图像：
rosrun rqt_image_view rqt_image_view /camera/rgb/image_raw
```

依赖 `gazebo_ros`、`gazebo_plugins`、`gazebo_msgs`、`rospy` 和 `tf2_ros`。图片板默认中心 (-2,2,0.02)，振幅 2m、周期约 12s；可通过 `amplitude`、`target_x/y/z` 调整，`move_target:=false` 停止移动。资源在 `worlds/`、`models/tracking_target/`，无需下载在线模型。此入口提供视觉环境，后续无人机仿真需另接 SITL/MAVROS。

靶标位置、运动范围和速度在 [config/sim_world.yaml](config/sim_world.yaml) 中设置：`x/y/z` 为 Gazebo 世界坐标系中的运动中心（m），`amplitude` 为 Y 轴往返振幅（m），移动范围为 `[y-amplitude, y+amplitude]`。例如 `speed: 0.5` 表示最大速度 0.5 m/s，`speed: 0.0` 表示在中心静止。靶标保持正弦往返轨迹，瞬时速度会变化，周期由 `T = 2π × amplitude / speed` 自动计算。配置中的位置和振幅优先于对应 launch 参数，`speed` 优先于旧 `period` 参数；删除对应配置项后可继续使用 launch 参数。修改后重启仿真生效，`sim_world.launch` 和 `sim.launch` 均支持 `config:=/absolute/path/to/custom.yaml` 指定配置。相机随无人机运动，调整靶标位置时需确认机身下视相机能看到目标。详见 [速度配置交付文档](docs/done/sim_world_speed_delivery.md)。

使用步骤、图片替换和 world 编写方法见 [仿真教程](docs/simWorld_tutorial.md)，变更与建议见 [交付文档](docs/simWorld_delivery.md)。

使用 `ref/sim.launch` 接入 PX4 时，启动文件必须将 `$(find maxt_pkg)/models` 加入 `GAZEBO_MODEL_PATH`，否则新 world 中的 `model://tracking_target` 无法从本包解析。该参考入口已补充此设置并保留原有 PX4 模型路径；它依赖外部 PX4、mavlink_sitl_gazebo 和 MAVROS 包。

当前 world 在 (0,0) 放置 `models/A_H/A_H.jpg` 起飞标记；三个障碍物中心 XY 分别为 (2,2)、(4,1)、(4,3)，尺寸均为 0.5×0.5×2m。目标在 (-2,0) 与 (-2,4) 之间往返，图片朝上，板面高度 0.02m。world 不再包含固定图像相机。`ref/sim.launch` 默认起飞位置仍为 (0,0)，已配置目标运动插件。详见 [场景修改交付](docs/modifyWorld_delivery.md)。

`launch/sim.launch` 默认使用 PX4 的 `iris` 飞控配置和 Gazebo 的 `iris_downward_depth_camera` 模型，机身下视相机彩色图像话题为 `/camera/rgb/image_raw`；已移除原 magpie360 模型对应的 Mid360 建图、里程计转发和静态 TF 链。定位采用 PX4 仿真传感器估计，通过 MAVROS 获取。此次减少了激光和建图负载，实际流畅度仍需本机运行确认。

靶标现由 Gazebo 原生插件按每个物理步更新，消除了原 Python 30Hz 同步服务调用造成的不均匀更新。首次使用需重新编译：`cd ~/catkin_ws && catkin_make --pkg maxt_pkg`，然后 `source devel/setup.bash`。`move_target`、`target_x/y/z`、`amplitude` 参数继续有效，`period` 在配置未指定 `speed` 时有效；`sim_world.launch` 的旧 `rate` 参数保留但不再控制更新频率。修复说明见 [靶标运动交付文档](docs/target_motion_delivery.md)。

## 视觉引导节点

`Guidance` 向 `/guidance` 的 `maxt_pkg/GuidanceAction` 服务提交 `target_id`，读取可见性和图像偏差，通过 PID 发布水平速度，保持开始时的高度和偏航。onRunning 内通过 WAIT_SERVER、WAIT_FEEDBACK、GUIDING、REACHED 状态推进任务。误差达标后恢复 HEARTBEAT；等待反馈和目标丢失时使用 HEARTBEAT 模式发布当前位置。

识别 action 服务需由感知端提供。XML 端口（仅 target_id、tolerance、timeout）、内部坐标映射和使用示例见 [Guidance 交付文档](docs/guidanceNode_delivery.md)。编译依赖新增 actionlib、actionlib_msgs 和 ROS 消息生成工具；修改后需重新运行 `catkin_make --pkg maxt_pkg` 并加载 `devel/setup.bash`。

模拟反馈测试：已启动仿真时运行 `roslaunch maxt_pkg guidance_test.launch`；也可运行 `roslaunch maxt_pkg guidance_test.launch run_sim:=true` 一起启动仿真。行为树起飞后发送 `target_id=test`，服务以 20Hz 发布固定 `error_u=20、error_v=0`，10 秒后停止反馈，Guidance 在 15 秒整体超时后执行降落。参数 `error_u`、`duration`、`rate` 可在 launch 命令中调整；模拟反馈与实际运动无关。详细步骤见 [测试脚本说明](docs/guidance_test_delivery.md)。

## 视觉伺服测试

运行 `roslaunch maxt_pkg vision_test.launch`：启动 PX4/Gazebo、机身下视相机识别与标注图像窗口，使用 `config/mission/vision_test.xml`。飞机起飞到 2m，巡航至 (-2, 2, 2)，悬停 2 秒后视觉跟踪 ArUco ID 0 共 60 秒，最后降落。视觉阶段最多等待 90 秒；丢失目标或图像过期时暂停水平控制，重新识别后继续跟踪。

靶标白边板为 0.5×0.5m，内部 ArUco 黑色方块为 0.375m。输入 `/camera/rgb/image_raw`，标注图像 `/aruco_center/debug_image`，真实像素误差通过 `/guidance` Action 提供给飞控。`Guidance` 的 `downward_camera=true` 将下视相机像素轴按当前航向转换到水平速度，`kp=0.003`，速度上限 0.5m/s；`tracking_duration=0` 保持原有单次对中行为。

靶标运动由 `config/sim_world.yaml` 控制。修改尺寸或配置后需重启仿真；修改 C++ 控制节点后需重新编译。

## UGV 轻量仿真

`script/ugv_sim.py` 使用 Matplotlib 显示小车运动与路径，不依赖 PX4 或 Gazebo。初始位置为 `(0,0)`，通过 `/ugv_mover` 接收现有 `ugv_mover.action` 的绝对 XY 目标（米），以恒定速度直线前进。速度在 `config/ugv_confg.yaml` 的 `speed` 中配置（m/s，正数）；文件名按需求保留 `confg` 拼写。

`script/ugv_controller.py` 按 `config/ugv_target.yaml` 的 `targets` 列表依次发送目标，终端显示状态；失败或被抢占后停止发送。Ctrl+C 取消当前目标，小车停在当前位置；新的 Action 目标可抢占正在执行的目标。关闭绘图窗口会退出仿真。

依赖 ROS Noetic Python 环境、已生成的 Action 消息、Matplotlib、PyYAML、rospkg 和桌面显示环境。首次使用需编译本包并加载工作空间环境：

```bash
cd ~/catkin_ws
catkin_make --pkg maxt_pkg
source devel/setup.bash
roscore
```

另开两个终端，加载相同工作空间环境后分别运行：

```bash
rosrun maxt_pkg ugv_sim.py
rosrun maxt_pkg ugv_controller.py
```

两端可通过 `_config:=/absolute/path/to/config.yaml` 分别指定速度配置与目标配置，也可通过 `_action_name:=/custom_ugv_mover` 设置相同服务名。此仿真使用墙钟推进，无需 `/clock`；运行时应使用默认 `use_sim_time=false`。配置修改后重启对应节点生效。

## 矩形区域遍历规划

`RegionTraverse` 接收按边界顺序排列的四个角点 `reg_point_1_x/y` 至 `reg_point_4_x/y` 和平行轨迹间距 `coverage_step`（米），生成蛇形路线。相邻航带严格按该间距排列，横向余量对称留在两侧；支持旋转矩形。

输出 `waypoints` 为 `maxt::RegionWaypoints`（`std::vector<std::array<double, 2>>`），`waypoint_count` 为 `int`，通过 XML `{黑板键}` 映射使用。节点只计算二维航点，不控制飞行；高度及后续航点执行由调用方负责。示例见 [region_traverse_test.xml](config/mission/region_traverse_test.xml)，接口、验证和建议见 [交付文档](docs/done/regionTraverse_delivery.md)。

`region_traverse_test.xml` 现为完整飞行任务：连接检查 → 起飞到 4m → 为 `(0,0)、(-6,0)、(-6,6)、(0,6)` 按 `coverage_step=1m` 生成 12 个航点 → `ReactiveFallback` 优先检查 `FoundCheck`，未找到时由 `Repeat` 配合 `GetRegionWaypoint` 逐点调用 `Nav`（z=4m）→ 找到目标后跟踪 60 秒再降落，遍历完成仍未找到则直接降落。编译并加载环境后执行：

```bash
roslaunch maxt_pkg region_traverse_test.launch
```

该入口会实际执行飞行；航带 y 为 0.5、1.5、2.5、3.5、4.5、5.5m，未找到目标时遍历全部航点，在 `(0,5.5)` 降落；找到目标时中断当前导航，转入 Guidance 跟踪，跟踪结束或失败后降落。

区域遍历入口默认复用已启动的 MAVROS 和定位系统，支持 `run_mavros`、`run_fast_lio`（默认 false）、`bt_tick_rate`、`heartbeat_rate`（默认 20Hz），与现有导航测试入口一致。

Nav 的 `v_max`、`a_max`、`pid_kp`、`pid_ki`、`pid_kd` 统一在 `config/nav.yaml` 中配置，由 `mission.launch` 加载至节点私有命名空间 `nav/`，不再支持 XML InputPort。可使用 `roslaunch maxt_pkg mission.launch bt_xml_path:=任务XML路径 nav_config:=自定义YAML路径` 指定配置文件。直接运行节点时需自行加载这些私有参数；参数缺失或无效时 Nav 返回 FAILURE。


## 区域遍历查找目标

`FoundCheckNode` 的头文件和源文件直接放在各自的 `maxt_nodes` 目录中，注册名为 `FoundCheck`。它订阅现有识别脚本的 `std_msgs/Bool`，只检查状态，不执行图像识别或飞行控制。

| XML 输入端口 | 默认值 | 含义 |
| --- | --- | --- |
| `topic` | `/aruco_center/is_found` | 检测话题，建树时订阅，不支持运行中切换 |
| `timeout` | `0.5` | 消息接收后有效时长，单位为秒，使用墙钟时间 |

最新消息为 true 且未过期时返回 SUCCESS；未收到消息、最新消息为 false 或超时均返回 FAILURE。该节点不锁存检测结果，不向黑板写入内容。任务 XML 在命中后通过 SetBlackboard 将 `region_search_result` 写为 `found`，遍历全部航点后写为 `complete`，由 Switch2 决定是否执行跟踪。`ReactiveFallback` 每个 tick 优先检查目标，命中时 halt 下方 Repeat 和正在运行的 Nav，复用 Nav 现有的 `onHalted()` 释放导航控制。仍沿用原任务的失败处理：Nav 失败时任务返回 FAILURE，不进入后续降落。

`region_traverse_test.launch` 默认启动 `aruco_center.launch`，并通过 `run_guidance:=true` 开启 `/guidance` Action 服务，提供 Guidance 所需的视觉误差反馈。`run_guidance:=false` 可关闭该服务；本任务的跟踪阶段需要有外部服务替代。相机、MAVROS 和定位系统默认由外部提供。已有识别节点时使用 `run_detection:=false`，并确保该节点已开启 `/guidance` 服务；需要启动相机时使用 `run_camera:=true`。`detection_config:=/绝对路径/参数.yaml` 可指定识别配置；若修改 `is_found_topic`，需同步修改 XML 的 FoundCheck `topic`；若修改 `marker_id`，需同步修改 Guidance 的 `target_id`。现有感知脚本识别的是配置指定的 ArUco 字典和 ID，FoundCheck 本身不解析通用 QR 码。

按使用者要求，最终运行测试由人工完成。建议依次确认：未找到时完成 12 个航点后降落；导航过程中找到目标时停止遍历、跟踪 60 秒后降落；检测为 false 或消息断流超过 0.5 秒时继续搜索。编译并加载工作空间环境后使用上面的 `region_traverse_test.launch` 入口。

跟踪阶段用 ForceSuccess 包裹 Guidance，使跟踪超时或 Action 失败后仍进入 TouchDown；`guidance_message` 和 Guidance 日志保留失败原因。此时最终任务 SUCCESS 表示降落流程完成，并不单独证明跟踪成功。未找到目标时跳过 Guidance；一旦搜索命中，后续短暂丢失由 Guidance 自身的等待反馈逻辑处理，不会恢复遍历。

## MavKit 曲线控制

`CURVE_CTRL` 模式由 MavKit 定时器采样五次曲线，直接发布位置、速度和加速度参考。
外部调用 `setQuinticCurveTarget(Eigen::Vector3d(x, y, z))` 设置世界坐标系目标，
调用 `getCurveTime()` 获取估算时长，再切换到 `SetpointMode::CURVE_CTRL`。
时长沿用 `nav/v_max`、`nav/a_max` 的估算公式，终点速度和加速度为零，新增依赖 Eigen3。
参考 `CurveState` 不随传感器更新；在 CURVE_CTRL 中重设目标时继承切换时刻的参考 p/v/a，
从其他模式进入时以当前位置和零速度、零加速度初始化。非零初始导数的重规划不保证严格限速。

Nav 在曲线执行期间仅检查总超时；曲线完成后先填充目标位置和三轴 PID 速度修正，
再切换 RAW_CTRL 进行末端对齐。达到容差或末端等待超过 3 秒返回 SUCCESS；
完成或超时时切回 HEARTBEAT；halt 时请求 SLOW_DOWN 平滑减速。

`SLOW_DOWN` 是独立的设点模式，MavKit 继承当前曲线参考 p/v/a，复用现有 `QuinticCurve`，
自由决定停车位置，由定时器的 SLOW_DOWN 分支采样和发布。
原有 CURVE_CTRL 分支只采样并发布导航曲线，不处理减速完成或自动模式切换。配置 `nav/slow_down_time`（默认 1.0 s）
和 `nav/stop_velocity_tolerance`（默认 0.01 m/s）；参数须为正。
所有轴完成减速、参考速度满足阈值且加速度归零后进入 HEARTBEAT。
判断使用参考状态，不代表实测飞机已经完全停止。该曲线不强制限制 jerk 或峰值加速度；
初始加速度与速度同向时，为保持加速度连续，速度可能先增加再下降。

减速未完成时，新 Nav 目标继承当前参考状态生成五次曲线；CONTROL 目标也转为五次曲线。
RAW_CTRL 接管使用最新设点中启用的速度轴作为过渡目标，忽略的速度轴按零处理，
使用五次插值平滑过渡至目标速度和零加速度，然后发布最新 RAW 设点。
因此 Guidance 等待反馈时的 HEARTBEAT 请求不会截断减速，收到反馈后可平滑接管参考速度。
该 RAW 过渡适用于本项目世界坐标系、非力控制且目标加速度为零或忽略的设点；
过渡期间发布完整 p/v/a，结束时恢复 RAW 的掩码、位置和 yaw，不对位置及 yaw 的切换做平滑。
STANDBY 仍立即停止发布。

## 空地协同搜索与返航

`ugv_cowork_test.xml` 复用区域遍历任务：连接 → 记录起飞位置 → 起飞 4m →
按 1m 间距搜索 `(0,0)、(-6,0)、(-6,6)、(0,6)` → 找到标记后视觉对齐 →
并行持续伺服与小车跟随 → 小车到达后停止伺服 → 返回起飞点上方 → 降落。
识别沿用现有 ArUco ID 0，并非通用 QR 码识别；原 `region_traverse_test.xml` 保留。

`Guidance` 的代码和接口保持不变。并行分支用 `KeepRunningUntilFailure` 包裹 Guidance，
将其成功转换为 RUNNING；`Parallel success_threshold="1" failure_threshold="1"`
因此仅由小车成功结束，自动 halt Guidance 并取消视觉 Action。
并行阶段小车总超时 90s，Guidance 跟踪 120s、总超时 150s，正常情况下不会中途重启跟踪。

`UgvCowork follow_drone="true"` 直接读取无人机自身 XY（米）作为小车目标，
不发布或修改无人机设点。每隔至少 0.2s 检查并发送变化超过 0.2m 的位置；
自己的更新由服务端抢占旧目标，不当作协同失败；外部客户端抢占则返回失败。
仅当最新 Action 为 SUCCEEDED、结果 `succeed=true`、无人机距已发送目标不超过 0.2m，
且该目标已保持至少 1s 未被替换时返回 SUCCESS。
这是“小车到达无人机位置”的判断，并不保证此刻视觉误差也达标；
初次对齐发生在小车跟随之前。没有进行图像到地面位置投影，要求双方坐标系一致。

| UgvCowork 输入端口 | 默认值 | 含义 |
| --- | --- | --- |
| `action_name` | `/ugv_mover` | 小车 Action 服务名 |
| `timeout` | `60.0` | 等待连接与到达的总墙钟秒数 |
| `follow_drone` | `false` | true 持续跟随无人机 XY；false 保留原单目标悬停模式 |
| `update_period` | `0.2` | 两次目标更新的最小墙钟间隔，秒 |
| `target_tolerance` | `0.2` | 目标更新及成功时的位置偏差阈值，米 |
| `settle_time` | `1.0` | 跟随模式最终目标的最小保持时长，墙钟秒 |

输出 `message` 为 WAIT_SERVER、WAIT_UGV、UGV_REACHED 或失败原因。
halt、超时和析构时仅取消本节点仍在执行的小车目标。

新增 `CapturePose` 将收到的定位快照写入黑板 `home_x/home_y/return_z`；
`height_offset=4.0` 使返航及搜索高度等于记录时地面 Z 加 4m。
该节点等待首条带时间戳的定位，默认超时 10s，不控制飞行。
头文件和源文件均直接位于各自的 `maxt_nodes/` 目录。
搜索矩形仍采用固定地图坐标，不随起飞点平移。

未找到目标、搜索失败、对齐失败或协同超时都会转入返航。
黑板 `cowork_result` 保存 not_found、search_failed、alignment_failed、cowork_failed 或 ugv_reached；
`alignment_message`、`guidance_message`、`ugv_message` 保存对应节点的细节。
返航使用现有 GoTo，实测位置进入各轴 0.15m 容差才成功；
返航超时则停止任务并保持位置，不进入降落。起飞失败也直接结束任务。
最终任务 SUCCESS 表示返航降落完成，协同是否成功以 `cowork_result` 为准。

编译并加载环境后，先准备无人机仿真/MAVROS 和定位，再运行：

```bash
roslaunch maxt_pkg ugv_cowork_test.launch
```

入口默认启动 ArUco 检测、视觉 Action 服务和 Matplotlib 小车仿真。
已有小车服务时使用 `run_ugv_sim:=false`；已有感知服务时使用 `run_detection:=false`。
支持 `detection_config`、`run_camera`、`run_mavros`、`run_fast_lio` 和两个频率参数。
`run_guidance:=false` 仅在另有 `/guidance` 服务时使用。
不要同时运行 `ugv_controller.py`，它的目标会抢占协同任务。
修改识别 ID 或话题后需同步修改 XML 中的 Guidance/FoundCheck 配置。
