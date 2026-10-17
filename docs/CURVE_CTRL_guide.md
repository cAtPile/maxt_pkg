CURVE_CTRL_guide.md

在setpointmode新增 CURVE_CTRL 模式，在mavkit中直接生成速度曲线，并直接用速度曲线控制飞机

## Supreme Principle
请阅读 SupremePrinciple.md

## 实现方式
1. 维护一个 CurveState 是曲线的状态而非传感器的
    ```
    struct CurveState {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
    };
    ```
2. 新增public函数 setQuinticCurveTarget(Eigen::Vector3d) 由外部调用填充目标位置
3. 新增public函数 getCurveTime() 得到曲线估算的时间
4. 在 loadCurveRaw函数 计算当前的 raw 并填充 curve_raw_target_
5. 切换到 CURVE_CTRL 模式时，发布 curve_raw_target_
6. 同时修改 NavNode 
7. 在 start 设置setQuinticCurveTarget 并 getCurveTime 将模式切换到 CURVE_CTRL
8. 在 running 中判断是否超出 CurveTime ，超出后使用原来的PID和强制SUCCEED逻辑
    先填充 rwa 然后切换到 RAW_CTRL 进行末端对齐 最后切换回 HEARTBEAT
