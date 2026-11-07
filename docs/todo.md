1. 整理 mission launch
2. 起飞子树（分段起飞）
3. 降落子树（分段降落）
5. 重构nav
6. 偏航调整
7. 打标签例如：TAKEOFF-INIT之类
8. 统一节点注册
9. 高度fix

创建一个新的Node：ReHeadingNode
参考nav的五次曲线和末端pid以及强制成功逻辑
让飞机航向恢复到azimuth_angle
