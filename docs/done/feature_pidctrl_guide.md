# feature_pidctrl_guide

添加pid控制模块，本模块主要在曲线控制末端进行末端对其用

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 实现方式
1. 添加新的SetpointMode::PID_CTRL
2. loadPIDRaw() 装载PID输出 pid_raw_target_
3. setPIDTarget 外部接口设置pid目标