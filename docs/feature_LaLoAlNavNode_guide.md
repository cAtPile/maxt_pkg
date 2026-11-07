# feature_LaLoAlNavNode_guide
经度纬度高度航行

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 前提条件
1. 在 get_pose.cpp 中创建 GetGlobalPose 返回经纬度信息
2. 在 set.cpp 中创建 SetGlobalRawTarget 用来设置经纬度的raw信息
3. 在 reach_check.cpp 中创建 ReachGlobalCheck 来检查经纬度是否到达
    其中tolerance由config做统一参数导入
4. 新增 SetpointMode::GLOBAL_RAW_CTRL 并加入到timer中

## 实现方法
借用原NavNode的方式
