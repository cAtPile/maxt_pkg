# Mavros eXtra function based on Behavior Tree

`maxt_pkg` 使用 BehaviorTree.CPP v3 编排 MAVROS 无人机基础飞行任务。

MaxtCore 负责初始化、注册行为树节点和执行 XML；MavKit 封装状态订阅、坐标标定、设点发布以及解锁和模式服务。任务节点使用已有非阻塞生命周期。

当前注册节点：`ConnectCheck`、`Takeoff`、`GoTo`、`QuinticNav`、`WaitStep`、`Hover`、`Round`、`TouchDown`、`Guidance`。`Land`、`XNavigation` 仅有历史头文件声明，未注册，不能用于任务 XML。

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
| `navigation_test.xml` | `roslaunch maxt_pkg navigation_test.launch` | QuinticNav 依次航行至 (0,0,1)、(2,0,1)、(2,2,1)、(0,0,1)，各点悬停 |
| `round_test.xml` | `roslaunch maxt_pkg round_test.launch` | 航行至 (0,0,1)，绕 (1,0,1) 顺时针一圈，半径 1m、角速度 20°/s，返回 (0,0,1) |

坐标单位为米，使用 MavKit 当前坐标系中的绝对坐标；这两个运动测试假设起飞点 XY 为 (0,0)，目标高度 z 为 1.0。若定位原点或地面高度不同，请先修改 XML 中的航点、圆心和高度。ConnectCheck 默认不启用坐标标定。导航限速为 0.8m/s，限加速度为 0.5m/s²；圆周加减速时间为 2s，超时为 45s。三个入口均支持相同的外部组件和频率开关。

QuinticNav 现有实现会在末端等待超过 3s 后强制返回 SUCCESS，因此行为树成功日志不能单独证明航点精度达标；请结合实际轨迹和误差检查结果。

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
