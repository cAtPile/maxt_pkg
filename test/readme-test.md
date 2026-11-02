
## 文件结构
```
test/
├── unit/                         # 纯算法，不连接 ROS master
│   ├── test_calibrate_tools.cpp   # 坐标旋转及逆变换
│   ├── test_quintic_curve.cpp     # 五次曲线边界条件
│   ├── test_pid_controller.cpp    # PID、限幅、重置
│   └── test_fast_mission.py       # 从现有位置迁入，测试 XML 生成
│
├── bt/                           # 行为树与动作状态机
│   ├── test_takeoff_node.cpp
│   ├── test_nav_node.cpp
│   ├── test_touch_down_node.cpp
│   ├── test_guidance_node.cpp
│   ├── test_region_traverse.cpp
│   └── test_ugv_cowork_tree.cpp
│
├── integration/                  # 真实 ROS 通信 + 模拟外部节点
│   ├── test_mavkit_topics.cpp
│   ├── test_mavkit_services.cpp
│   ├── test_setpoint_transition.cpp
│   ├── test_guidance_action.cpp
│   └── test_mission_executor.py
│
├── sitl/                         # PX4 + MAVROS + Gazebo
│   ├── test_takeoff_land.py
│   ├── test_navigation.py
│   ├── test_visual_guidance.py
│   └── test_ugv_cowork.py
│
├── support/                      # 测试替身与公共工具
│   ├── fake_vehicle.hpp          # 模拟状态、记录控制请求
│   ├── fake_clock.hpp            # 手动推进时间
│   ├── mock_mavros.py            # 模拟话题、服务与状态反馈
│   ├── mock_guidance_server.py
│   ├── mock_ugv_server.py
│   └── assertions.hpp           # 坐标、设点、轨迹断言
│
├── fixtures/                     # 小型、固定的测试输入
│   ├── missions/                 # 最小 XML、失败分支 XML
│   ├── configs/                  # 测试专用参数
│   └── vision/                   # 少量 ArUco 测试图片
│
└── launch/                       # rostest 启动文件
    ├── mavkit.test
    ├── guidance.test
    └── mission_executor.test
    
```