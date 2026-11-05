# feature_azimuthFixNavNode_guide
方位角固定系航行

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 实现方式
1. 输入端口
    1. azimuth_angle: 角度制，是要固定的航行角
    2. nav_n: 沿着方向角航行的距离
    3. nav_t: 垂直于方向角航行的距离
    4. timeout
    5. tolerance
2. 我希望实现的效果是：
    获取航行参数和当前位置

## 前置条件
1. Mavkit::GetRawPose 获取未经处理的坐标
2. Mavkit::GetHomeOffsetPose 获取抵消起飞点的坐标，但没有角度标定
3. Mavkit::GetHomeCalibratePose 获取角度矫正后的且抵消起飞点的坐标
4. Mavkit::GetCalibratePose 获取角度标定后的坐标，但没有抵消起飞点


