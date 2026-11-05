# refactor_touchDown_guide
降落逻辑重构

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 前提
1. MavKit::RecordRawHomePose() 起飞前记录未经处理的 pose
2. MavKit::GetRawHomePose() 返回起飞前未经处理的 raw_home_pose

## 实现方法
1. 通过 GetRawPose 获得 xy
2. 通过 GetHomePose 或者 home_z
3. 其他逻辑保持原有