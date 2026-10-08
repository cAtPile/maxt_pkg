# Iris 仿真入口调整

修改 ref/sim.launch 的默认 SDF 为 mavlink_sitl_gazebo/models/$(arg vehicle)/$(arg vehicle).sdf，默认 vehicle=iris。此前 vehicle 虽为 iris，实际加载的仍是 tutorial_gazebo 的 magpie360.sdf。

移除 mapping_mid360.launch、odom_forward_node 和对应 body/base_link、odom/camera_init、map/odom 静态 TF，避免运行与 Iris 模型不匹配的激光建图管线。保留 PX4 SITL、MAVROS、现有 world 和移动目标。

入口不再提供原建图 TF 与外部视觉里程计转发，定位由 PX4 模型的仿真传感器与估计器提供。README 已更新。launch XML 解析通过，本机 PX4 目录存在 iris.sdf；未运行仿真或测量帧率。

核心建议：

1. 重启实际使用的 sim.launch，若运行外部副本需同步修改 SDF 和移除建图节点。
2. 运行任务前检查 MAVROS 连接状态与 local_position 数据。
3. 若仍卡顿，可关闭 GUI 并检查 Gazebo 实时因子，区分渲染和仿真计算负载。
