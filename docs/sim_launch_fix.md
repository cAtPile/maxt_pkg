# 更换 world 后的模型路径修正

`ref/sim.launch` 使用 tracking_test.world，但原入口未配置本包模型路径。最近 ROS 启动日志中的 GAZEBO_MODEL_PATH 也没有 maxt_pkg/models。新 world 通过 model://tracking_target 引入图片板，因此存在本地模型无法解析的问题。

已在 PX4 include 之前添加 GAZEBO_MODEL_PATH，将本包 models 加入搜索路径并保留继承路径。未更改飞控、建图和 MAVROS 接口。README 同步补充说明。

launch XML 解析通过；未重新启动 PX4/Gazebo，尚不能断言完整启动故障全部解决。该入口也未启动图片移动控制器，加载场景与目标自动运动是两个独立步骤。

建议：

1. 使用修改后的 ref/sim.launch 重新启动；若使用外部副本，将同一 env 配置加入实际启动文件。
2. 保留原有 PX4 和 tutorial 模型路径，避免飞机模型资源丢失。
3. 若仍失败，提供实际启动命令及终端第一条错误，以继续定位 Gazebo 或 PX4 的其他问题。
