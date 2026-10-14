# UGV 移动协同交付

依据 `cowork-ugv_guide.md` 与 `SupremePrinciple.md`，新增一个通过 Action 受控移动的 UGV demo。未修改指南、原 UGV 控制代码或无人机模块。

## 修改内容与接口

- 新增 `action/ugv_mover.action`：目标 `float32 target_x/target_y`，结果 `bool succeed`，反馈 `uint8 state`，常量 WAITING、STARTING、MOVING、REACHED。指南中的状态拼写和 `.pu` 文件后缀规范化。
- 新增 `ugv_demo/ugv_move_demo.py`：Action 服务端，默认名称 `/ugv_mover`；处理目标、取消/抢占、成功、超时和控制丢失，保持等待后续任务。
- 新增 `ugv_demo/ugv_move_controller.py`：Action 客户端，从 `ugv_demo/config/ugv_move.yaml` 选择目标列表，成功后顺序执行；失败停止，退出时取消当前任务。
- 新增 `ugv_demo/ugv_move.launch`：启动服务端和可选客户端，支持配置路径、目标方案、速度、容差、超时、MAVROS 命名空间和航向补偿开关。
- 更新 CMake 消息生成和脚本/配置安装声明，更新项目与 UGV README。未移除既有接口；ReadyCheck 与完整 MissionTarget 信息流不属于本次简单 demo。

## 设计

服务端继承 `RoverDemo`，复用状态/定位订阅、20Hz 速度发布、闭环距离控制、旧 PX4 速度坐标补偿和停止处理。Action 执行线程负责启动就绪检查、请求 OFFBOARD/解锁、任务状态以及结果；独立定时器持续发布速度。启动时输出零速度，移动阶段进入 MOVING，到容差内返回 REACHED 与成功，终止前清零。反馈异常和任务超时返回 ABORTED，取消或新任务抢占返回 PREEMPTED。失败原因通过 actionlib 状态文本给出，结果字段仍保持指南规定的简单布尔值。

坐标为 MAVROS 本地 ENU 绝对坐标。当前闭环为直接目标跟踪，未实现障碍物避让或最小转弯半径规划。到达表示位置进入容差，不代表车辆已完全静止。取消时停止非零设点，但不主动上锁。

## 检查与人工验证

仅做 Python/launch/YAML/Action 静态检查，未编译消息或启动仿真，运行验证由用户进行。步骤见 `ugv_demo/README.md`：编译加载环境、启动仿真且关闭旧控制节点、启动 Action demo。建议检查 `(3,3)` 到达成功、two_points 连续任务、移动中取消及新目标抢占、反馈丢失或超时返回失败。

## 三条核心建议

1. 同一 MAVROS 实例保持唯一速度发布节点，避免旧 demo 与 Action 服务端争用设点。
2. 首先确认本机 PX4 的航向补偿与到点误差，再扩展目的地；修复固件后关闭 legacy_rover_velocity。
3. 后续实际空地协同优先补充就绪检查与车辆转弯/避障规划，再接入投放或无人机任务触发。

## 启动导入修复

修复 `ugv_move_demo.py` 从 catkin devel 包装脚本启动时导入同目录 `ugv_offboard.RoverDemo` 失败的问题。服务端通过源文件 `__file__` 将同目录加入 Python 搜索路径，优先导入原控制源码；安装布局中两个脚本同目录，仍适用。使用实际 devel 包装脚本以非主程序模式确认导入成功，未初始化 ROS 节点或启动仿真。此 Python 源码修复无需重新编译，重启 `ugv_move.launch` 生效。
