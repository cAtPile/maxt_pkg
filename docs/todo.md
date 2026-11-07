1. 整理 mission launch
2. 起飞子树（分段起飞）
3. 降落子树（分段降落）
4. fix下牛耕
5. 重构nav
6. 偏航调整
7. 打标签例如：TAKEOFF-INIT之类
8. 统一节点注册
9. 高度fix

## fix下牛耕
1. 起飞6m
2. LaLo La=47.3977419 Lo=8.5455939
3. BasePointAzimuthFix
4. TraversePathGenerator 明确生成的是什么轨迹
现在我有了新的节点
LaLoNav 和 AzimuthFixNav
我想让区域遍历可以以一个确定的经纬度为起点并以AzimuthFixNav固定的方向为坐标x正方向，生成遍历航迹。
然后使用 AzimuthFixNav航行。我盖如何设计，仅仅设计无需编写具体代码
