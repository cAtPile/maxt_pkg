# refactor_getPose_guide
起飞节点重构

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 实现方式
1. 获取 GetRawPose 的 xyz
2. 根据原有的逻辑 将 GetRawPose 的xy保持 加上恒定的垂直速度 takeoff_raw_target
3. setRawTarge(takeoff_raw_target)
4. 设置setpointmode为RAW_CTRL
5. 将GetRawPose的z+目标高度=aim_altitude_raw
6. 调用ReachAltitudeCheck(aim_altitude_raw)

## modify
修改原来的等待解锁的逻辑，取消超时检测，持续等待
