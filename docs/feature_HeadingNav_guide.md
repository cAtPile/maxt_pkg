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

## find
```
[INFO] [1793802654.341942977, 21.657000000]: TakeoffNode: OFFBOARD confirmed, home=(-0.00, 0.01, -0.22)
[INFO] [1793802654.392155112, 21.707000000]: TakeoffNode: Ascending from z=-0.22 to z=0.78 at 0.50 m/s, yaw=-0.08
INFO  [commander] Takeoff detected
[INFO] [1793802664.878245516, 32.059000000]: TakeoffNode: Reached target altitude (0.79 >= 0.78)!
[1793802664.926]: Takeoff                   RUNNING -> SUCCESS
[INFO] [1793802664.928225232, 32.109000000]: [HeadingNav] START: target=(0.28,-0.73,0.81) T=2.88
[1793802664.928]: HeadingNav                IDLE -> RUNNING
[INFO] [1793802668.943235774, 35.857000000]: [HeadingNav] REACHED: pos=(0.373, -0.736, 0.794) pos_err=0.097 elapsed=3.75s
[1793802668.943]: HeadingNav                RUNNING -> SUCCESS
[INFO] [1793802668.946663360, 35.860000000]: [HeadingNav] START: target=(-0.08,0.16,0.79) T=2.88
[1793802668.947]: HeadingNav                IDLE -> RUNNING
[INFO] [1793802671.852916826, 38.757000000]: [HeadingNav] REACHED: pos=(-0.074, 0.168, 0.797) pos_err=0.010 elapsed=2.90s
[1793802671.853]: HeadingNav                RUNNING -> SUCCESS
[INFO] [1793802671.855654559, 38.760000000]: [HeadingNav] START: target=(0.63,-0.55,0.80) T=2.88
[1793802671.856]: HeadingNav                IDLE -> RUNNING
[INFO] [1793802675.110294773, 42.007000000]: [HeadingNav] REACHED: pos=(0.714, -0.509, 0.781) pos_err=0.098 elapsed=3.25s
[1793802675.110]: HeadingNav                RUNNING -> SUCCESS
[INFO] [1793802675.113403508, 42.010000000]: [HeadingNav] START: target=(-0.17,-0.04,0.78) T=2.88
[1793802675.114]: HeadingNav                IDLE -> RUNNING
[INFO] [1793802678.017025435, 44.908000000]: [HeadingNav] REACHED: pos=(-0.189, -0.057, 0.762) pos_err=0.032 elapsed=2.90s
[1793802678.017]: HeadingNav                RUNNING -> SUCCESS
[INFO] [1793802678.019545380, 44.910000000]: [HeadingNav] START: target=(0.79,-0.23,0.76) T=2.88
[1793802678.020]: HeadingNav                IDLE -> RUNNING
[INFO] [1793802681.028026510, 47.807000000]: [HeadingNav] REACHED: pos=(0.808, -0.150, 0.762) pos_err=0.084 elapsed=2.90s
[1793802681.028]: HeadingNav                RUNNING -> SUCCESS
[INFO] [1793802681.031501983, 47.810000000]: [HeadingNav] START: target=(0.02,-0.77,0.76) T=2.88
[1793802681.032]: HeadingNav                IDLE -> RUNNING
[INFO] [1793802684.040624882, 50.807000000]: [HeadingNav] REACHED: pos=(0.043, -0.862, 0.777) pos_err=0.100 elapsed=3.00s
[1793802684.041]: HeadingNav                RUNNING -> SUCCESS
[INFO] [1793802684.044495571, 50.811000000]: TouchDownNode: Start, home_z=-0.22, descent_speed=-0.30, timeout=60.0, ground_timeout=3.0, height_tol=0.30, vel_tol=0.20
[1793802684.045]: TouchDown                 IDLE -> RUNNING
[INFO] [1793802684.089960491, 50.858000000]: TouchDownNode: Descending from (0.04, -0.86) at -0.30 m/s
```

## question
1. 轨迹混乱
2. 记得取消角度标定
3. 注意位置