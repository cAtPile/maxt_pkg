1. 整理 mission launch
2. 起飞子树（分段起飞）
3. 降落子树（分段降落）
5. 重构nav
6. 偏航调整
7. 打标签例如：TAKEOFF-INIT之类
8. 统一节点注册
9. 高度fix

rostopic pub -r 20 /QRCode_pose/Global_pose \
sensor_msgs/NavSatFix \
"header:
  stamp: now
  frame_id: 'gps'
latitude: 47.397832
longitude: 8.545594
altitude: 5.0"

