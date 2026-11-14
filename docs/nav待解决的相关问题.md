以当前 NavNode 为例，PID 计算虽然已经迁入 MavKit，但还存在以下耦合：

1. PID 配置仍由两层共同管理，最值得先清理。
   Nav 还在读取、校验并保存 pid_kp/ki/kd，但实际控制器由 MavKit 再次读取配置。两边策略也不同：Nav 缺参数直接失败，MavKit 使用默认值。应统一由控制层管理配置与有效性检查。见 nav.cpp:53 (src/maxt_nodes/nav/nav.cpp:53)、pid.cpp:22 (src/mavkit/pid.cpp:22)。
2. Nav 依赖 MavKit 内部曲线的时间规则。
   Nav 获取 getCurveTime() 后，用自己的 traj_start_ 判断曲线结束；MavKit 另有 curve_start_。同一个执行过程维护两套时钟。更合理的是由 MavKit 提供曲线完成状态，Nav 据此进入末端阶段。
3. 设置目标与切换模式存在调用顺序耦合。
   Nav 必须先调用 setPIDTarget()，再切换 PID_CTRL，且只能在进入末端时设置一次，否则积分会重置。调用方需要了解控制器生命周期。可以封装为一次“开始末端对齐”操作。见 nav.cpp:120 (src/maxt_nodes/nav/nav.cpp:120)。
4. 坐标系约定隐含在调用流程中。
   输入坐标先加 home，误差使用 GetCalibratePose()，PID 目标则依赖 raw_local=false 的默认值。这些必须保持一致，但接口没有明确表达坐标类型。
5. 公共基类实现放在 Nav 源文件里。
   MavActionNode::recordBreakpointAndSlowDown() 被多个导航节点使用，却定义在 nav.cpp，使公共行为依赖具体节点的编译单元。适合移到基类对应源文件。见 nav.cpp:6 (src/maxt_nodes/nav/nav.cpp:6)。
6. navgeo GEO_CTRL 下沉
7. T_ 可以被整合

另外，到达阈值、总超时、末端超时后返回 SUCCESS 属于行为树任务策略，留在 Nav 层是合理的，不必全部下沉