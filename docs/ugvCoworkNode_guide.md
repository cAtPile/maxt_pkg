# ugvCoworkNode_guide

空地协同节点

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 实现方式
1. 创建一个新的bt节点，无人机使用 ugv_mover.action 发布位置
2. ugv到达位置后节点成功退出位置
3. 构建测试用的任务xml，起飞，飞行到(4,3) 待小车到达位置后，降落
