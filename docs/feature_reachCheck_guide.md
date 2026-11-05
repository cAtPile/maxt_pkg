# feature_reachCheck_guide
统一到达检查窗口

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 实现方式
1. 建立 include/maxt_pkg/mavkit/reach_check.hpp
2. 建立 src/mavkit/reach_check.cpp在其中建立
3. 新增 bool MavKit::ReachAltitudeCheck( double altitude_raw ) 用来检测高度是否到达
4. 新增 bool MavKit::ReachPoseCheck( geometry_msgs::PoseStamped pose_raw ) 用来检测点是否到达
5. 新增 config 参数
    1. vertical_alt_tolerance 垂直高度容忍值
    2. vertical_vel_tolerance 垂直速度容忍值
    3. position_pose_tolerance 点位置容忍值
    4. position_vel_tolerance 点速度容忍值
6. 针对 ReachAltitudeCheck 
    获得当前高度 RawPose 中的 z
    检测高度是否符合输入值
    当高度误差符合 vertical_alt_tolerance
    且垂直速度误差符合 vertical_vel_tolerance
    时，返回正确
7. 针对 ReachPoseCheck 
    检测位置是否符合输入值
    获得当前位置 RawPose 中的 x,y,z
    计算当前 RawPose 和输入 pose_raw 的欧式距离
    计算当前的和标量速度
    当位置误差符合 position_pose_tolerance
    且当速度标量符合 position_vel_tolerance 时
    返回正确
