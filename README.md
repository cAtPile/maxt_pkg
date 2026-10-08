# Mavros eXtra function based on Behavior Tree

`maxt_pkg` 使用 BehaviorTree.CPP v3 编排 MAVROS 无人机基础飞行任务。

MaxtCore 负责初始化、注册行为树节点和执行 XML；MavKit 封装状态订阅、坐标标定、设点发布以及解锁和模式服务。任务节点使用已有非阻塞生命周期。

当前注册节点：`ConnectCheck`、`Takeoff`、`GoTo`、`QuinticNav`、`WaitStep`、`Hover`、`Round`、`TouchDown`。`Land`、`XNavigation` 仅有历史头文件声明，未注册，不能用于任务 XML。

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

新增 ROS Noetic / Gazebo Classic 11 场景，包含地面、三个方柱障碍物、沿 Y 轴往返的图片板和固定相机：

```bash
roslaunch maxt_pkg sim_world.launch
rosrun rqt_image_view rqt_image_view /sim_world/camera/image_raw
```

依赖 `gazebo_ros`、`gazebo_plugins`、`gazebo_msgs`、`rospy` 和 `tf2_ros`。图片板默认中心 (-2,2,0.02)，振幅 2m、周期 12s；可通过 `amplitude`、`period`、`target_x/y/z` 调整，`move_target:=false` 停止移动。资源在 `worlds/`、`models/tracking_target/`，无需下载在线模型。此入口提供视觉环境，后续无人机仿真需另接 SITL/MAVROS。

使用步骤、图片替换和 world 编写方法见 [仿真教程](docs/simWorld_tutorial.md)，变更与建议见 [交付文档](docs/simWorld_delivery.md)。

使用 `ref/sim.launch` 接入 PX4 时，启动文件必须将 `$(find maxt_pkg)/models` 加入 `GAZEBO_MODEL_PATH`，否则新 world 中的 `model://tracking_target` 无法从本包解析。该参考入口已补充此设置并保留原有 PX4 模型路径；它依赖外部 PX4、mavlink_sitl_gazebo 和 MAVROS 包。

当前 world 在 (0,0) 放置 `models/A_H/A_H.jpg` 起飞标记；三个障碍物中心 XY 分别为 (2,2)、(4,1)、(4,3)，尺寸均为 0.5×0.5×2m。目标在 (-2,0) 与 (-2,4) 之间往返，图片朝上，板面高度 0.02m。固定相机在 (-2,2,6) 俯视地面。`ref/sim.launch` 默认起飞位置仍为 (0,0)，已配置目标运动插件。详见 [场景修改交付](docs/modifyWorld_delivery.md)。

`ref/sim.launch` 默认使用 PX4 的 `iris` 模型，SDF 与 `vehicle` 参数一致；已移除原 magpie360 模型对应的 Mid360 建图、里程计转发和静态 TF 链。定位采用 PX4 仿真传感器估计，通过 MAVROS 获取。此次减少了激光和建图负载，实际流畅度仍需本机运行确认。

靶标现由 Gazebo 原生插件按每个物理步更新，消除了原 Python 30Hz 同步服务调用造成的不均匀更新。首次使用需重新编译：`cd ~/catkin_ws && catkin_make --pkg maxt_pkg`，然后 `source devel/setup.bash`。`move_target`、`target_x/y/z`、`amplitude`、`period` 参数继续有效；`sim_world.launch` 的旧 `rate` 参数保留但不再控制更新频率。修复说明见 [靶标运动交付文档](docs/target_motion_delivery.md)。
