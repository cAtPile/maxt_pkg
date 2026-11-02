# 大规模重构测试文件计划

## 目标与范围

在改动实现之前建立可重复运行的行为基线，保护飞控设点、行为树生命周期、ROS 接口及任务编排。本文是文件与用例计划，尚未创建或执行这些新增测试。

测试应断言输入与可观察输出，不依赖私有成员布局、源码字符串或某个辅助函数必须被调用。将“保持现有行为”的特征测试与“修复已确认问题”的回归测试分开；可疑行为先记录，不直接当成永久正确的规范。

## 现有基础

- `test/test_fast_mission.py`：已有端口/默认值一致性、XML 生成、区域子树组合、CLI、参数编辑、TUI 布局和监看测试。保留并扩展；其中解析 C++ 源码的端口检查较脆弱，后续可改用 BT 工厂导出的运行时端口清单。
- `test/ugv_cowork_tree_test.cpp`：已有替身叶节点驱动真实协同 XML 的流程测试，覆盖发现目标、未发现、搜索/对齐/引导/协同/返航失败及跟踪提前成功。它验证树的编排，不验证真实控制节点。
- CMake 当前在 `CATKIN_ENABLE_TESTING` 下注册协同行为树测试；Python 测试目前需要手动运行。

## 测试层次与基础设施

| 层次 | 运行环境 | 用途 |
| --- | --- | --- |
| 纯单元测试 | gtest 或 Python unittest，无 ROS master | 数学、规划、消息构造、编辑器逻辑 |
| ROS 集成测试 | rostest + 独立 ROS master + 假 MAVROS/Action 服务 | 真实 MavKit、节点状态机、回调、超时、取消 |
| XML 流程测试 | BT 工厂 + 替身叶节点 | 子树黑板、失败传播、halt、并行结束条件 |
| SITL 验收 | 外部 PX4 + Gazebo + MAVROS | 坐标方向、实际轨迹与完整任务；按需运行 |

建议增加以下公共文件，避免每份测试重复搭建环境：

| 文件 | 职责 |
| --- | --- |
| `test/support/fake_mavros.py` | 发布 state、extended_state、pose、twist；提供 arming/set_mode 服务；记录两类设点与服务请求，允许脚本化延迟/失败。不能自动把每个设点当作实际位置，以免掩盖未到达问题。 |
| `test/support/fake_action_servers.py` | 可编程 Guidance/UGV Action 服务，支持等待、反馈、成功、失败、抢占及取消记录。 |
| `test/support/ros_fixture.hpp` | 节点构造、消息收集、带墙钟上限的条件等待、清理与参数恢复。 |
| `test/support/bt_test_nodes.hpp` | 从现有协同测试按需提取替身节点、事件记录和场景注入。 |
| `test/fixtures/` | 最小任务 XML、固定 YAML、视觉测试图像；输入固定可复现，不放大体积录包。 |

当前节点依赖具体 MavKit，不能假定其方法可直接 mock。第一阶段通过真实 MavKit 与假 ROS 对端建立基线；若重构引入控制接口/可注入时钟，再逐步迁移为更快的单元测试，保留关键 ROS 集成用例。

## P0：重构开始前优先建立

| 测试文件 | 重点用例与断言 | 方式 |
| --- | --- | --- |
| `test/test_calibrate_tools.cpp` | 零旋转、±90°、任意角；位置/速度正逆变换恢复原值；raw 正向加速度旋转；Z、消息掩码及当前未变换字段保持；不同 map/fact 方向决定旋转符号 | gtest |
| `test/test_quintic_curve.cpp` | 三轴非零初末 P/V/A 边界；零位移；不同正时长；有限差分核对速度/加速度；轴之间不串扰。采用数学边界作为预期，不复制系数算法 | gtest |
| `test/test_pid_controller.cpp` | 正负误差、积分与输出限幅、reset、增益更新、dt 边界的现有语义；分别覆盖头文件中实际使用的控制器类型 | gtest |
| `test/test_mavkit_setpoints.cpp` | STANDBY 不发；HEARTBEAT 持续发当前位姿；CONTROL/RAW_CTRL 坐标变换；默认 yaw 覆盖；home 捕获与查询；设点时间戳；BT 不 tick 时心跳仍运行 | rostest |
| `test/test_mavkit_transitions.cpp` | CURVE_CTRL→SLOW_DOWN→HEARTBEAT；减速中接 RAW_CTRL/CONTROL；重复停止/相同速度目标不重新计时；切换前后参考 P/V/A 连续；结束速度和加速度满足容差 | rostest，之后可拆纯曲线测试 |
| `test/test_takeoff_node.cpp` | 未解锁仅等待且不主动 arm；等待超时；已解锁后 OFFBOARD 请求与确认；home/目标高度；爬升掩码与速度方向；完成、超时、halt、重复启动 | rostest |
| `test/test_touch_down_node.cpp` | 已上锁立即成功；下降控制；高度/垂速/落地状态联合确认；确认被打断重计时；请求上锁、确认及重试；全局超时分支；halt | rostest |
| `test/test_nav_node.cpp` | 非零 home 下目标偏移；缺少 PID 参数失败；轨迹期与末端 PID；到达容差；任务超时；末端超过 3 秒的现有成功行为；halt 触发减速；重复启动清状态 | rostest |
| `test/test_mission_contracts.cpp` | 用实际节点注册和实际端口创建仓库任务树，解析 include/SubTreePlus；检查黑板映射；只构造不 tick。生成的 fast_mission XML 使用临时目录单独测试 | gtest 或 rostest（取决于构造依赖） |

每个 ROS C++ 测试文件对应同名 `.test` 启动文件，例如 `test/test_nav_node.test`，启动所需替身及测试可执行程序。不同测试使用独立 master，避免绝对 `/mavros/*`、`/guidance` 等话题串扰。

## P1：对应模块重构前完成

| 测试文件 | 重点用例与断言 | 方式 |
| --- | --- | --- |
| `test/test_guidance_node.cpp` | 服务延迟；首次反馈；像素误差方向、下视旋转、限速、定高；相同反馈不重复积分；反馈过期/目标丢失；恢复；对齐与定时跟踪；服务终止、超时、halt 取消；旧请求反馈不能污染新执行 | rostest |
| `test/test_ugv_cowork_node.cpp` | 固定目标悬停；follow_drone 不写飞控；位移阈值和更新周期；新位置优先于旧目标成功；settle_time；失败/超时/取消；只取消自身目标 | rostest |
| `test/test_hover_wait_nodes.cpp` | Hover 固定启动位置而非追随实时位置；持续时间和 halt；WaitStep 等待但不写控制设点；两者再次启动重新计时 | rostest |
| `test/test_round_node.cpp` | 起点与半径；顺时针方向；PVA 数学关系；加速/匀速/减速边界；短圆弧；最终容差；超时与 halt。数学部分抽出后用纯单测，保留节点生命周期集成测试 | rostest + 后续 gtest |
| `test/test_connection_pose_nodes.cpp` | ConnectCheck 连接/定位准备、标定与超时；CapturePose 输出坐标和高度偏移、无定位超时；重复执行 | rostest |
| `test/test_region_waypoints.cpp` | 正负边长、非整除步距、窄区域、边界覆盖、蛇形顺序；GetRegionWaypoint 的索引推进与末端行为；检查空间性质及少量手算例子 | gtest |
| `test/test_region_subtrees.cpp` | 真实 RegionTraverse/VisionRegionTraverse XML：完整遍历、发现后 halt 导航、导航失败、result 值；同树多个子树实例黑板隔离 | BT 流程测试 |
| `test/ugv_cowork_tree_test.cpp`（扩展） | 保留既有场景，补充外部 halt、重复执行及黑板结果；验证并行另一分支被停止 | 现有测试扩展 |
| `test/test_found_check_node.cpp` | true/false、无消息、消息过期、新消息恢复；topic 参数与重新执行 | rostest |
| `test/test_maxt_core.cpp` | XML 加载失败、正常成功/失败、节点注册；mission_status JSON 字段、并行 active、无运行节点、最终状态；进程停止有界 | rostest，独立进程 |
| `test/test_audio_caller.cpp` | 服务缺失/拒绝/延迟；play 不阻塞主任务；忙时允许丢弃、无重试；调用中析构不等待；避免依赖线程调度断言精确丢弃数量 | rostest，独立子进程 |
| `test/test_play_audio_node.cpp` | 缺 filename 失败；每次执行提交一次；重启再次提交；服务失败不影响节点成功 | rostest |

## P2：工具、感知和端到端补齐

| 测试文件 | 重点用例与断言 | 方式 |
| --- | --- | --- |
| `test/test_fast_mission.py`（扩展） | 保留现有覆盖；编辑/撤销/排序/取消；临时文件保存与回读；重复任务和子树；关闭监看；不依赖源码排版 | unittest |
| `test/test_mission_monitor.py` | 从原文件按规模拆分：无 ROS、断连、过期、并行 active、缺省 active、最终状态保留；用可控时间避免真实等待 | unittest |
| `test/test_aruco_center.py` | 固定合成 marker 的 ID/中心像素误差；空白帧、多 ID、选择目标；检测状态与 Action 反馈一致；取消和目标切换 | 图像纯测试 + rostest |
| `test/test_ugv_sim.py` | 给定速度的位移、到达不越界、新目标抢占、取消、结果与反馈；测试环境关闭绘图，运动时间可控 | unittest/rostest |
| `test/test_ugv_controller.py` | 顺序发目标；成功后推进；失败停止；退出取消当前目标；配置路径覆盖 | unittest + 假 Action 对端 |
| `test/test_fake_audio.py` | 合法请求立即响应；新请求重计时；非法请求不打断；不使用真实音频设备 | rostest |
| `test/test_launch_config.py` | 展开 mission 系列 launch，检查 sim/run_uav/run_fake_audio 组合、参数命名空间与转发；不启动真实飞控；外部包依赖单独标识 | ROS 配置测试 |
| `test/test_sitl_missions.py` + `test/sitl_missions.test` | 起飞→导航→悬停→降落；区域扫描；发现→减速→引导→协同→返航；非零起点与标定角场景；记录轨迹、最终状态及失败原因 | 可选 SITL 验收 |

`min_snap.hpp`、旧节点及监视绘图脚本按实际引用和此次重构范围决定是否追加测试，不为未使用代码提前建立大量测试。

## 必须先明确的行为基线

以下情况单独登记为待确认项；先写能暴露差异的场景，不通过放宽断言掩盖问题：

1. Nav 末端对齐超时会强制 SUCCESS；需要决定重构保持还是另行修复。
2. TouchDown 全局超时可能每 tick 重新进入上锁分支，影响确认与重试计时。
3. CapturePose 输出与 Nav 的 home 相对输入组合，在非零 home 下是否重复叠加；必须覆盖真实值，原点场景无法发现。
4. `CalibrateTools::untransRaw(PositionTarget&)` 当前只逆变换位置和速度；不能直接假设加速度也满足正逆往返。
5. 普通 Sequence 中前序失败会跳过末尾降落；协同树的容错路径又不同，应分别验证。
6. 起飞/降落构造的 yaw 掩码不同，MavKit 还会应用默认 yaw；应断言最终发布消息。

## 实施顺序与通过标准

1. 先运行并记录现有测试结果、代码提交号和依赖版本；已有失败先解释，不让重构与问题修复混在一起。
2. 搭建假 ROS 对端并完成 P0。优先从真实话题/服务建立外部基线，再抽内部接口。
3. 按计划重构哪个模块，就先完成对应 P1/P2 用例；避免先大改再写迎合新实现的测试。
4. 将 C++ 单测用 `catkin_add_gtest` 注册，ROS 集成用 `add_rostest_gtest`/`add_rostest` 注册；Python unittest 接入测试目标，补齐 package.xml 测试依赖。这里是未来实施动作，本次未修改构建配置。
5. 每次提交运行纯单测、XML 流程和相关 ROS 集成；合并前跑完整自动化套件，涉及运动/坐标/控制切换时追加 SITL。

时间相关断言采用显式推进的 ROS 仿真时钟；墙钟逻辑在可注入前使用短时间参数及有界条件等待。禁止靠固定长 sleep 判断完成。数值采用绝对/相对容差，连续性比较参考设点而非要求真实机体瞬间满足导数边界。

所有测试要有退出上限、取消与清理，不能连接已有实机 ROS master；测试产物写临时目录，不覆盖用户任务 XML。SITL 容差根据基线测量确定，不凭空设定“全部轨迹必须精确重合”。

验收以关键行为覆盖为主：成功、失败、超时、halt、重复启动、消息过期及控制切换均有对应断言；测试能在无人值守环境稳定结束。覆盖率用于发现遗漏，不以单一百分比替代这些要求。
