# 重复逻辑与函数抽取建议

本清单基于当前源码静态检查，仅提出建议，未实施重构。定位使用文件与函数名，避免后续代码改动使行号失效。优先级综合重复程度、维护收益与行为变更风险；不把简单 API 调用重复一律视为需要抽象。

## 高优先级

### 1. 固定位置悬停设点的构造

- 位置：`src/maxt_nodes/hover.cpp` 的 `onStart()`、`onRunning()`；`src/maxt_nodes/ugv_cowork_node.cpp` 的 `onStart()`。
- 重复：初始化 `PositionTarget`，设置 map、LOCAL_NED、忽略速度/加速度/偏航角速度的掩码，再填入位置与 yaw。Hover 内部两处几乎相同。
- 建议：抽取 `makePositionHoldTarget(position, yaw)`，返回消息；由调用者决定是否切换模式。
- 收益：消除掩码和字段复制，统一固定点悬停的消息语义。
- 边界：不能替换为 HEARTBEAT；后者使用实时当前位置，固定点悬停使用启动时记录的位置。

### 2. 原始设点写入后接管控制

- 位置：`takeoff.cpp`、`touch_down.cpp`、`hover.cpp`、`round.cpp`、`guidance.cpp`、`ugv_cowork_node.cpp` 的启动/控制分支，以及 `nav.cpp::onRunning()`。
- 重复：`setRawTarget(target)` 后紧接 `setSetpointMode(RAW_CTRL)`。
- 建议：在 MavKit 中提供 `activateRawTarget(target)`，复用已有目标写入和模式切换逻辑。
- 收益：明确“先更新目标，再接管控制”的顺序，减少漏写或反序风险。
- 边界：保留仅更新目标的 `setRawTarget()`；Hover、Round 的运行阶段已有这种用途。不能直接赋值模式绕过 SLOW_DOWN 的平滑接管，也不能在持锁后调用再次加锁的公开方法。

### 3. 曲线边界装配与启动

- 位置：`src/maxt_mav/maxt_mav_request.cpp` 的 `setQuinticCurveTargetLocked()`、`startVelocityTransition()`。
- 重复：将 `curve_state_` 的三轴位置、速度、加速度展开为 `setBoundaries()` 参数，设置 `curve_start_`，再调用 `loadCurveRaw(now)`。
- 建议：抽取私有函数 `initializeCurveLocked(end_position, end_velocity, duration, now)`，终点加速度保持当前的零值。
- 收益：集中三轴边界映射，降低复制时轴顺序或导数填错的风险。
- 边界：时长估算、终点计算、`velocity_transition_` 和模式更新仍归各调用者负责；函数约定调用前已持锁。

## 中优先级

### 4. 竖直速度控制设点

- 位置：`src/maxt_nodes/takeoff.cpp::onRunning()` 的 INIT；`src/maxt_nodes/touch_down.cpp::onRunning()` 的 INIT。
- 重复：保持 XY，忽略 Z 位置、XY 速度和三轴加速度，设置 Z 速度。
- 建议：抽取 `makeVerticalVelocityTarget(x, y, vz, ...)`，显式表达航向控制策略。
- 边界：起飞使用 yaw，降落消息使用零 yaw_rate，两者掩码不完全相同；MavKit 默认航向锁定还会覆盖航向字段。不能直接统一成同一掩码而改变语义。

### 5. 三维位置误差与到达判断

- 位置：`src/maxt_mav/maxt_mav_cb.cpp::isReached()`、`src/maxt_nodes/nav.cpp::onRunning()`、`src/maxt_nodes/round.cpp::onRunning()`。
- 重复：计算 XYZ 差值及欧氏距离，再与容差比较。
- 建议：Round 可考虑直接复用已有 `isReached()`；需要误差日志的 Nav 可使用纯函数 `positionError(current, target)` 或距离函数。
- 边界：Nav 已读取当前位姿用于 PID 和日志，应复用同一快照，避免额外查询导致判断与日志不一致。Nav 的末端超时强制成功属于独立策略，不应塞入距离函数。
- 分类：这里属于已有抽象未充分复用，以及缺少可返回误差的公共函数。

### 6. yaw 转四元数并填入姿态

- 位置：`src/maxt_mav/maxt_mav_cb.cpp` 的 `heartbeatTimerCallback()` 内部 `publish_hold`，以及 `setTargetPose()`。
- 重复：创建 tf2 四元数，`setRPY(0, 0, yaw)`，逐项复制 x/y/z/w。
- 建议：抽取纯函数 `quaternionFromYaw(yaw)`，返回 `geometry_msgs::Quaternion`。
- 边界：保留心跳分支的 `has_default_yaw_` 条件，不改变无默认航向时沿用当前姿态的行为。

### 7. 降落进入上锁阶段

- 位置：`src/maxt_nodes/touch_down.cpp::onRunning()` 的全局超时分支、落地确认成功分支。
- 重复：切换 HEARTBEAT、请求上锁、记录请求时间、设置 `Phase::DISARM`。
- 建议：抽取节点私有函数 `beginDisarm(now)`，两处分支保留各自日志。
- 边界：此次抽取只合并动作。全局超时每次 tick 可能重复进入该分支的现有行为，需要独立审查，不能以重构名义悄悄修改。

### 8. 圆周设点的组装

- 位置：`src/maxt_nodes/round.cpp::onStart()`、`onRunning()`。
- 重复：构造同一掩码的 PVA 消息，写入圆周位置、速度、加速度及 yaw；启动阶段是角速度和角加速度为零的特例。
- 建议：抽取节点私有函数 `makeOrbitTarget(theta, omega, alpha)`，统一圆周公式和消息装配。
- 边界：保留顺时针符号约定；角度统一使用弧度，阶段切换、计时和完成判断留在原状态机。

## 低优先级，可按后续修改需要实施

### 9. 相同构造签名的行为树节点注册

- 位置：`src/maxt_core.cpp::registerNodes()`。
- 重复：多个 `registerBuilder<T>()` lambda 都调用 `make_unique<T>(name, config, *mav_)`。
- 建议：小型模板辅助函数 `registerMavNode<T>(id)`，只覆盖这一种构造签名。
- 边界：Nav 额外需要 NodeHandle，FoundCheck 使用 NodeHandle，两个路径节点直接注册；这些差异应保留。当前显式注册易读，收益主要是新增节点时少写样板。

### 10. Python 包内 YAML 配置读取

- 位置：`script/ugv_sim.py::UgvSimulation.__init__()`、`script/ugv_controller.py::main()`。
- 重复：RosPack 定位包、拼接默认配置路径、读取 `~config` 覆盖、UTF-8 打开文件、`yaml.safe_load()`。
- 建议：若后续增加使用者，抽取 `load_package_yaml(default_relative_path, param_name='~config')`。
- 边界：配置字段解析仍由各脚本负责；新增共享模块需同时处理 rosrun/catkin 安装后的导入路径。当前仅两处短代码，收益有限。

## 不建议仅因重复就统一的部分

- **节点超时判断**：ROS 时间与墙钟并存，起飞还区分等待解锁和飞行计时。单纯包装时间相减收益低，统一计时策略可能改变行为。
- **节点结束清理**：Guidance 和 UgvCowork 已有各自的 `finish()`；协同跟随模式不能干预无人机控制，Nav 中断需要减速，不宜统一成“所有节点结束都切心跳”。
- **ROS 参数读取与 BT 端口声明**：多数是领域配置声明，而非重复算法。过度泛化会隐藏默认值和参数归属。
- **坐标变换与五次曲线求解**：已有 `calibrate_tools.hpp`、`quintic_curve.hpp` 抽象；应优先复用现有工具，不另建平行实现。
- **测试中的消息和任务构造**：重复输入可能用于独立表达测试场景，不应与生产代码一起机械去重。

## 建议实施顺序

先处理固定点悬停消息、yaw 四元数和降落节点内部重复，变更范围较小；再处理原始设点接管与曲线初始化，重点核对锁、模式过渡和发布行为。每项单独修改与验证，不把本清单当作一次性大重构计划。
