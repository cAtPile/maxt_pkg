# cowork-ugv_guide

空地协同模块，本次只要实现一个简单的受控协同的ugv即可

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 实现内容
1. 创建ugv_mover.action
```
# ugv_mover.action

# goal
# 目标位置
float32 target_x
float32 target_y
---
# result
# 任务结果
bool succeed
# 失败原因（暂空）
---
# feedback
# 任务状态
{MOVINGING,WAITING,REACHED,STARING...}

```
2. 为小车创建一个 ugv_move_demo.py,接受ugv_mover.action消息流的控制。
3. 创建 ugv_move_controller.pu,发出控制指令。且控制的的目的地可以根据config切换