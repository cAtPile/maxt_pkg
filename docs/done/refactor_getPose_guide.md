# refactor_getPose_guide
获取点工具重构

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 实现方式
1. 创建新的 include/mavkit/get_pose.hpp 和 src/mavkit/get_pose.cpp 来管理点获取的逻辑
2. 具体包含的工具如下：
    1. Mavkit::GetRawPose 获取未经处理的坐标
    2. Mavkit::GetHomeOffsetPose 获取抵消起飞点的坐标，但没有角度标定
    3. Mavkit::GetHomeCalibratePose 获取角度矫正后的且抵消起飞点的坐标
    4. Mavkit::GetCalibratePose 获取角度标定后的坐标，但没有抵消起飞点
3. 创建后替换原有的函数