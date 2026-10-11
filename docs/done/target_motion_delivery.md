# 靶标运动卡顿修复

旧实现通过 moving_target.py 以约 30Hz 同步调用 /gazebo/set_model_state，并在每次调用时使用非持久服务连接。在 PX4 SITL 运行期间，跨进程调度和服务等待会使位置更新不均匀。Gazebo ROS 的 setModelState 实现还会临时暂停世界、写入姿态、恢复暂停状态。该调用链是代码层面确认的卡顿风险，未对用户原场景进行帧率测量。

新增 src/sim/moving_target_plugin.cpp 和 libmaxt_moving_target_plugin.so，在 WorldUpdateBegin 中按仿真时间计算正弦位置，不使用 ROS 服务、/clock 回调或墙钟定时器。ROS 参数只在模型加载时读取。每次更新保留 world 中的初始姿态，因而地面靶标持续朝上。世界重置和时间回退会重置相位。

tracking_target/model.sdf 挂载插件；sim_world.launch 与 ref/sim.launch 改为提供 /moving_target 参数，不再启动 Python 节点。CMake 增加插件构建与安装，package.xml 增加 gazebo_dev 构建依赖和 Gazebo 插件安装路径；README 和教程同步更新。

接口：move_target、target_x/y/z、amplitude、period 保留；旧 rate 参数兼容接收但不再控制物理步更新。原 Python 脚本仍保留，默认拒绝与原生插件同时运行；需要旧方式时必须先以 move_target:=false 重启。未改变飞控接口。

首次使用必须编译并 source 工作空间，然后重新启动实际使用的 sim.launch。若使用外部启动文件副本，应同步新的 moving_target 参数组并移除 Python node，避免重复控制。

验证：在 /tmp 的独立 catkin 构建目录完成 CMake 配置和插件目标编译；XML、Python 语法及 git diff --check 通过。独立无界面 Gazebo 世界运行 12001 个 1ms 步，目标 Y 范围为 0～4m，最大单步位移 0.0010472m，正弦轨迹最大误差 4.44e-16m；X/Z 与朝上姿态检查通过，世界重置后的轨迹误差为零。该测试显式加载模型插件，不含 PX4 或相机渲染，尚未测量联合仿真的 GUI 帧率。

参考：[Gazebo ROS setModelState 源码](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp#L1504)。

核心建议：

1. 启动时确认日志出现 MovingTargetPlugin，且没有 Failed to load plugin 错误。
2. 同一靶标仅使用一个控制器，参数通过 launch 设置并重启生效。
3. 相机仍为 30Hz，若图像卡顿但模型轨迹连续，应进一步检查渲染帧率和图像处理负载。

启动不动排查：实际 Gazebo 日志确认 `Failed to load plugin libmaxt_moving_target_plugin.so`，原因是插件此前仅在 /tmp 独立测试目录编译，实际 catkin 工作空间 devel/lib 缺少产物。应在实际工作空间构建 maxt_moving_target_plugin 目标并重新加载环境、重启 Gazebo。

已在实际 /home/a/catkin_ws 工作空间构建 maxt_moving_target_plugin，生成 devel/lib/libmaxt_moving_target_plugin.so，并通过共享库加载检查。用户需停止旧仿真、source devel/setup.bash 后重新启动。
