# feature_azimuthFixNavNode_guide
方位角固定航行

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 前置条件
1. Mavkit::GetRawPose 获取未经处理的坐标
2. Mavkit::GetHomeOffsetPose 获取抵消起飞点的坐标，但没有角度标定
3. Mavkit::GetHomeCalibratePose 获取角度矫正后的且抵消起飞点的坐标
4. Mavkit::GetCalibratePose 获取角度标定后的坐标，但没有抵消起飞点

## 实现方式
1. 在 config/nav.yaml 中添加 azimuth_angle 角度制，是要固定的航行角
2. 创建新节点 BasePointAzimuthFixNode 方位角固定基点
3. 创建新节点 AzimuthFixNavNode 方位角固定航行
4. 关于 BasePointAzimuthFixNode 有以下几点要点
    1. 记录当前位置为固定方位角航行基点
    2. tick一次运行
    3. 调用 GetRawPose 获得基点位置并记录在黑板
5. 关于 AzimuthFixNavNode 有以下几点要点
    1. 输入端口
        1. nav_n: 沿着方向角航行的距离
        2. nav_t: 垂直于方向角航行的距离
        3. nav_a: 垂直于基点的高度变化
        4. bp_x_raw: 基点的x坐标（未经标定，offest处理）
        5. bp_y_raw：基点的y坐标（未经标定，offest处理）
        6. bp_z_raw: 基点的z坐标（未经标定，offest处理）
    2. 我希望实现的效果是：
        1. 根据配置的 azimuth_angle 为本系的方位角0
        2. 以及输入的 bp_x/y/z_raw 为本系的(0,0,0)点
        3. 在本系中航行 nav_n/t/a
    3. 具体的实现方法是：
        1. 将nav_n/t投影到原来的坐标系算出需要航行的trans_pose的x/y/z
        2. 然后+bp_x/y/z_raw 成为实际上需要航行的pose_raw
        3. 然后模仿原nav的逻辑进行航行





