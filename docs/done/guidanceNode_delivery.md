# Guidance 引导节点交付

## 修改范围

新增 `action/Guidance.action`、`GuidanceNode`，注册 XML 节点 `Guidance`。
增加 actionlib、actionlib_msgs、message_generation/message_runtime 依赖及 action 生成、安装规则。
移除重复的 `SetpointMode::GUIDANCE`，引导等待统一复用 HEARTBEAT 发布当前位置。
原有节点接口保持不变；指南中的 `RWA_CTRL` 使用项目现有拼写 `RAW_CTRL`。
未修改指南文件和其他飞行节点。

## Action 接口

默认服务名 `/guidance`，类型 `maxt_pkg/GuidanceAction`。

- goal：`string target_id`。
- feedback：`bool target_visible`、`float32 error_u`、`float32 error_v`。
- result：`string message`，用于描述 SUCCEED、FAIL、TIMEOUT 等结果。

本次实现是行为树 action 客户端；目标识别端需提供同类型 action 服务，持续反馈选定目标的偏差。
不会自动创建识别算法，也不会从 Gazebo 图片中直接提取目标。

## 使用示例

在已完成连接检查和起飞的任务序列中加入：

```xml
<Guidance target_id="tracking_target"
          tolerance="5.0"
          timeout="30.0"
          message="{guidance_message}" />
```

XML 只需设置 `target_id`、`tolerance` 和 `timeout`。`tolerance` 是每个图像轴的容差，期望偏差 `bias_u/v` 在代码中默认为 0。
反馈可以用像素，也可以用归一化坐标；bias、tolerance 和 PID 增益必须与其单位一致。
`timeout` 包含连接服务、等待反馈和调整的总时间，单位为秒。
action 名称固定为 `/guidance`；反馈有效期、PID 增益、积分限幅、速度上限和图像到世界坐标的映射均为节点内部配置。
当前默认值分别为反馈有效期 0.5 秒、PID 增益 0.01/0/0、积分限幅 50、速度上限 0.5m/s，u/v 轴按单位矩阵映射。
需要调整这些设置时，修改 `GuidanceNode` 的成员默认值后重新编译。
输出端口 `message` 给出 RUNNING、SUCCEED、TIMEOUT、HALTED 或具体失败原因。

## 生命周期和坐标约定

onStart 创建客户端、读取端口并切换 HEARTBEAT；已连接则发送 goal。
若 actionlib 的连接握手尚未完成，则在 onRunning 非阻塞等待并发送，等待计入 timeout。
`onRunning()` 内使用 `switch (guidance_state_)` 推进状态机：

| 状态 | 行为与切换条件 |
| --- | --- |
| WAIT_SERVER | 等待 action 服务连接，发送 goal 后切换 WAIT_FEEDBACK |
| WAIT_FEEDBACK | 保持 HEARTBEAT，收到可见且未过期的反馈后进入 GUIDING |
| GUIDING | 每条新反馈计算一次 PID，发布 RAW_CTRL；丢失目标回到 WAIT_FEEDBACK；误差达标进入 REACHED |
| REACHED | 取消 goal，恢复 HEARTBEAT，返回 SUCCESS |

超时和 action 失败作为全局终止条件优先检查。
收到第一条有效反馈时，同一个 tick 即进入 GUIDING 处理；MavKit 定时持续发布 RAW_CTRL 设点。
两个残差 `error_u - bias_u`、`error_v - bias_v` 均在 tolerance 内即返回 SUCCESS、取消 goal 并恢复 HEARTBEAT。
目标丢失或反馈过期时进入 HEARTBEAT 并重置 PID，重新获得目标后继续控制。
服务 abort/preempt/reject 等终态返回 FAILURE；服务 SUCCEEDED 也必须有可见、新鲜、达标的反馈才能成功。
服务结果文本用于失败描述，不能单独替代图像误差判定。
超时和 onHalted 均取消本节点 goal 并恢复 HEARTBEAT。

默认输出为 `PID(bias, error)`，即 kp 为正时误差为正产生负向控制。
图像与世界轴的关系由四个内部变量组成二维映射：

```text
vx = u_to_x * PID_u + v_to_x * PID_v
vy = u_to_y * PID_u + v_to_y * PID_v
```

默认 u_to_x=1、u_to_y=0、v_to_x=0、v_to_y=1。
这只是初始映射，指南未给出相机外参；需要按实际相机方向调整符号、交换轴或旋转矩阵。
输出进入 MavKit 的世界/标定坐标系，经现有 calib_.transRaw 转换。
使用 MAVROS local raw 接口的 ENU 输入语义，保留开始时高度和偏航，仅控制水平速度。
HEARTBEAT 沿用现有当前位置心跳语义。
本节点不负责解锁、切换 OFFBOARD 或起飞。

## 编译和验证

```bash
cd ~/catkin_ws
catkin_make --pkg maxt_pkg
source devel/setup.bash
```

在 `/tmp` 独立构建生成 action 并编译任务执行器，避免覆盖已有工作空间产物。
action 代码生成和 maxt_test_node 完整编译通过；随后实际 `~/catkin_ws` 工作空间的目标编译也通过，新库、执行器和 action 类型已生成到 devel。
初版独立 ROS master 和模拟 action 服务验证四种情况均通过：可见误差收敛成功、无反馈超时、服务 abort 失败、目标丢失后恢复位置保持。
同时验证速度方向、合成速度限幅和开始时高度保持。XML 解析和 git diff --check 通过。
本次状态机修改重新进行编译检查；未重新进行运行测试。实际相机映射和闭环稳定性仍需仿真调试。

## 三条核心建议

1. 外部代码若使用 SetpointMode::GUIDANCE，改用 HEARTBEAT 并重新编译。先用模拟 feedback 确认图像轴到世界轴的方向，再以较低 max_speed 在仿真中调整 PID。
2. 识别服务必须持续反馈 target_visible；失去目标也应发送不可见状态，避免依赖超时才停止控制。
3. 明确使用像素还是归一化误差；更换分辨率或相机后同步调整 bias、tolerance 和 PID 增益。
