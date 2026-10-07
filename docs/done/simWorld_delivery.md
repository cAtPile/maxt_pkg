# simWorld 交付说明

新增 `worlds/tracking_test.world`：自包含地面、光源、方箱、立柱、墙体及固定 ROS 相机；新增 `models/tracking_target/`：1m 方形图片板、UV 网格、双面材质和几何特征图片。

新增 `script/moving_target.py`：使用仿真时间和 Gazebo SetModelState 服务驱动图片板沿 Y 轴正弦往返，支持范围、周期、频率及中心位置参数。暂停时停止，仿真时钟回退时重启运动相位。使用服务响应检查目标存在性。

新增 `launch/sim_world.launch`：配置模型路径，启动 Gazebo、移动控制器和相机光学 TF。新增 `/sim_world/camera/image_raw`、`/sim_world/camera/camera_info` 图像接口及 `world -> tracking_camera_optical_frame` 静态 TF，使用 Gazebo 标准 `/gazebo/set_model_state` 服务。未移除或修改现有飞行接口。

更新 CMake 安装规则与运行依赖，使脚本、world、模型和图片在 install 空间也可使用；更新 README，新增 [world 编写和使用教程](simWorld_tutorial.md)。本次未修改指南和飞行任务模块。

设计将视觉场景与飞控解耦，场景可独立检查跟踪输入，再接入无人机。板采用静态模型加位置控制，适合可重复视觉测试，不模拟受力运动。相机视场覆盖默认运动范围，障碍物避开默认视线；图案不依赖比赛识别模块。

验证结果：Python 语法和 XML 解析通过，world 与图片板模型均通过 Gazebo 原生 `gz sdf -k` 校验，`git diff --check` 通过。Gazebo 实际渲染、跟踪算法与飞行联调未执行。

核心建议：

1. 先通过相机话题确认图片清晰、往返完整，再接入跟踪器，并记录识别误差和延迟。
2. 调整图片、速度或相机位置时保持运动范围在视场内，同时同步更新相机 TF。
3. 飞行仿真接入同一 Gazebo 实例，先确认 SITL/MAVROS 与定位，再运行既有飞行任务。
