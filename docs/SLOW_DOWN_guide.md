# SLOW_DOWN_guide.md

在setpointmode新增 SLOW_DOWN 模式，在中断后平滑速度

## Supreme Principle
请阅读 SupremePrinciple.md

## 实现方式
1. 切换到 SLOW_DOWN 模式时，设置目标速度为0，0，0,终点位置不需要固定，各轴以quinticcurve减速即可，生成曲线然后切换到 CURVE_CTRL
2. 到速度绝对值符合要求后进入HEARTBEAT
3. SLOW_DOWN 模式被打断后依然可以保持速度的平滑
