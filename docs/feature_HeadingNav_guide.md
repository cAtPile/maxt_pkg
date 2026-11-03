# feature_HeadingNav_guide

航向固定航向模式添加

## Supreme Principle
请阅读 SupremePrinciple.md

## 注意
不修改本文件

## 实现方式
1. 建立新的节点 HeadingNavNode 的bt树节点
2. 输入端口分别为 heading_angle 航向偏航角; nav_dis 航行距离; tolerance timeout
3. 根据输入端口数据，飞机沿该航向航行指定距离
4. 可参考NavNode进行设计
