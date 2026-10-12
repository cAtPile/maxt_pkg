# simworld 靶标速度配置交付

新增 `config/sim_world.yaml`，用 `speed` 设置靶标最大移动速度（m/s），默认约 1.0472，保持原先振幅 2 m、周期 12 s 的运动。两个仿真入口 `sim_world.launch`、`sim.launch` 新增 `config` 参数，将 YAML 读入 `/moving_target` 命名空间；安装规则包含 YAML 文件。

原生运动插件读取速度，利用 `T = 2πA/v_max` 计算周期，继续按物理步更新正弦轨迹。速度为 0 或振幅为 0 时停在配置中心；负数、非有限或非数值速度会记录错误并停止启用运动。配置在加载插件时读取，修改后重启生效。

未移除接口。配置存在 `speed` 时优先使用它；配置没有 `speed` 时沿用原 `period` 参数。旧 Python 手动控制脚本仍使用其原有周期参数。

追加位置配置：YAML 现包含 `x/y/z`（世界坐标系运动中心，m）和 `amplitude`（Y 轴振幅，m）。默认中心为 (-2, 2, 0.02)，振幅为 2，保持原有运动范围。复用插件已有参数读取逻辑，YAML 在 launch 参数后加载，因此同名配置项优先；删除配置项即可使用对应 launch 参数。位置与振幅修改无需再次编译，重启仿真即可生效。固定相机仍使用 world 中的位置。

使用示例：将 YAML 改为 `speed: 0.5`，重新编译插件并加载工作空间环境后运行 `roslaunch maxt_pkg sim_world.launch`。自定义配置可用 `config:=/absolute/path/to/custom.yaml`，PX4 入口同样支持。

验证：插件通过本机 ROS Noetic / Gazebo 11 头文件的 C++ 语法编译检查；检查两个 launch 的 XML 及 YAML 配置，并核对速度与周期换算。未启动 Gazebo 或执行完整 catkin 构建。

建议：

1. 根据跟踪算法能力逐步提高 `speed`，先从 0.3–0.5 m/s 开始。
2. 调整振幅时检查目标是否仍在相机视野内；相同最大速度下，振幅越大周期越长。
3. 用仿真时间衡量速度，避免将机器卡顿造成的墙钟时间变化误认为运动配置错误。
