# maxt_pkg 文件结构

以下按目录列出包中的文件，省略隐藏项。

```text
maxt_pkg/
├── action/                                      # ROS 动作接口，定义任务目标、反馈和结果
│   ├── Guidance.action
│   └── ugv_mover.action
├── config/                                      # 导航、地面车与仿真参数
│   ├── mission/                                 # 任务行为树配置
│   │   ├── subtree/                             # 主任务调用的行为子树
│   │   │   ├── GuidanceRecoveryTree.xml
│   │   │   ├── NavgeoDetectTree.xml
│   │   │   ├── RollbackTargetTree.xml
│   │   │   ├── TraverseDetectTree.xml
│   │   │   └── TraverseRecoreTree.xml
│   │   ├── testFlight_1114/
│   │   └── traverse_guidance_mission.xml        # 编排区域遍历与引导任务的主行为树
│   ├── aruco_center.yaml
│   ├── lidar_voxels.rviz
│   ├── nav.yaml
│   ├── sim_world.yaml
│   ├── ugv_confg.yaml
│   └── ugv_target.yaml
├── docs/                                        # 开发文档、功能说明和任务记录
│   ├── done/                                    # 已完成的功能修改记录
│   │   ├── audio_modify.md
│   │   ├── feature_LaLoAlNavNode_guide.md
│   │   ├── feature_azimuthFixNavNode_guide.md
│   │   ├── feature_reachCheck_guide.md
│   │   ├── modify_NavNode_guide.md
│   │   ├── refactor_getPose_guide.md
│   │   ├── refactor_takeoff_guide.md
│   │   └── refactor_touchDown_guide.md
│   ├── SupremePrinciple.md
│   ├── add_new_bt_node.md
│   ├── cowork-infoStream.md
│   ├── cowork-ugv_guide.md
│   ├── definition.md
│   ├── feature_pidctrl_guide.md
│   ├── found_check.md
│   ├── git_push_proxy.md
│   ├── nav待解决的相关问题.md
│   ├── new_timer_guide.md
│   ├── pkg_structor.md
│   ├── todo.md
│   ├── tui_plan.md
│   ├── uav_audio_接入说明(1).md
│   └── ugvCoworkNode_guide.md
├── include/                                     # C++ 头文件，声明飞控、导航和行为树接口
│   └── maxt_pkg/
│       ├── mavkit/                              # 飞控工具接口，涵盖位姿读取、控制与到点判断
│       │   └── maxt_mav.hpp
│       ├── maxt_nodes/                          # 起飞、导航、降落和协同等行为树节点
│       │   ├── nav/
│       │   │   ├── nav_azimuth_node.hpp
│       │   │   ├── nav_geo_node.hpp
│       │   │   ├── nav_node.hpp
│       │   │   ├── nav_raw_node.hpp
│       │   │   └── xnav_node.hpp
│       │   ├── base_point_azimuth_fix_node.hpp
│       │   ├── capture_pose_node.hpp
│       │   ├── connect_check_node.hpp
│       │   ├── found_check_node.hpp
│       │   ├── get_region_waypoint_node.hpp
│       │   ├── guidance_node.hpp
│       │   ├── hover_node.hpp
│       │   ├── land_node.hpp
│       │   ├── list_recorded_targets_node.hpp
│       │   ├── play_audio_node.hpp
│       │   ├── re_heading_node.hpp
│       │   ├── recorded_targets_node.hpp
│       │   ├── region_waypoints.hpp
│       │   ├── round_node.hpp
│       │   ├── send_geo_pose_node.hpp
│       │   ├── takeoff_node.hpp
│       │   ├── target_check_node.hpp
│       │   ├── touch_down_node.hpp
│       │   ├── traverse_path_generator_node.hpp
│       │   ├── ugv_cowork_node.hpp
│       │   └── waitstep_node.hpp
│       ├── audio_caller.hpp
│       ├── calibrate_tools.hpp
│       ├── global_nav_curve.hpp
│       ├── maxt_core.hpp
│       ├── maxt_nodes.hpp
│       ├── min_snap.hpp
│       ├── pid_controller.hpp
│       └── quintic_curve.hpp
├── launch/                                      # ROS 启动文件，用于组合节点并加载参数
│   ├── aruco_center.launch
│   ├── competition_world.launch
│   ├── lidar_grid.launch
│   ├── lidar_sim.launch
│   ├── lidar_voxel_map.launch
│   ├── mavros.launch
│   ├── mission.launch                           # 任务启动入口
│   ├── sim.launch
│   └── sim_geo_treverse.launch
├── models/                                      # Gazebo 模型及其网格、材质和纹理
│   ├── A_H/
│   │   ├── materials/
│   │   │   └── scripts/
│   │   │       └── marker.material
│   │   ├── A_H.jpg
│   │   ├── model.config
│   │   └── model.sdf
│   ├── A_aqr/
│   │   └── A_aqr.png
│   ├── iris_downward_depth_camera_lidar/
│   │   ├── model.config
│   │   └── model.sdf
│   ├── targets/
│   │   └── materials/
│   │       ├── scripts/
│   │       │   └── targets.material
│   │       └── textures/
│   │           ├── dict_4x4_50_id0.png
│   │           ├── dict_4x4_50_id1.png
│   │           ├── dict_4x4_50_id2.png
│   │           ├── dict_4x4_50_id3.png
│   │           ├── dict_4x4_50_id4.png
│   │           └── dict_4x4_50_id5.png
│   └── tracking_target/
│       ├── materials/
│       │   ├── scripts/
│       │   │   └── target.material
│       │   └── textures/
│       │       ├── aruco_4x4_50_id0.png
│       │       └── target.png
│       ├── meshes/
│       │   └── panel.dae
│       ├── model.config
│       └── model.sdf
├── nodes/                                       # 激光雷达处理和测试用 ROS 节点源码
│   ├── lidar_cloud_adapter.cpp
│   ├── lidar_grid.cpp
│   └── maxt_test_node.cpp
├── script/                                      # 视觉识别、地面车控制与仿真用的 Python 脚本
│   ├── __pycache__/                             # Python 自动生成的字节码缓存
│   │   ├── aruco_center.cpython-38.pyc
│   │   └── ugv_sim.cpython-38.pyc
│   ├── fast_mission/
│   │   ├── __pycache__/                         # Python 自动生成的字节码缓存
│   │   │   └── fast_mission.cpython-38.pyc
│   │   └── fast_mission.py
│   ├── tools/                                   # 速度与平面轨迹监测脚本
│   │   ├── xvel_monitor.py
│   │   └── xytraj_monitor.py
│   ├── aruco_center.py
│   ├── fake_audio.py
│   ├── fake_reciver.py
│   ├── guidance_test_server.py
│   ├── moving_target.py
│   ├── simple_camera_driver.py
│   ├── ugv_controller.py
│   └── ugv_sim.py
├── src/                                         # 飞控、行为树节点与仿真插件的 C++ 实现
│   ├── bt_core/
│   │   └── bt_registry.cpp                      # 注册可在任务 XML 中使用的行为树节点
│   ├── mavkit/                                  # 飞控工具接口，涵盖位姿读取、控制与到点判断
│   │   ├── callback.cpp
│   │   ├── get_pose.cpp
│   │   ├── pid.cpp
│   │   ├── reach_check.cpp
│   │   ├── set.cpp
│   │   └── timer.cpp
│   ├── maxt_mav/
│   │   ├── maxt_mav_cb.cpp
│   │   ├── maxt_mav_con.cpp
│   │   └── maxt_mav_request.cpp
│   ├── maxt_nodes/                              # 起飞、导航、降落和协同等行为树节点
│   │   ├── nav/
│   │   │   ├── nav.cpp
│   │   │   ├── nav_azimuth.cpp
│   │   │   ├── nav_geo.cpp
│   │   │   └── nav_raw.cpp
│   │   ├── base_point_azimuth_fix.cpp
│   │   ├── capture_pose_node.cpp
│   │   ├── connect_check.cpp
│   │   ├── found_check.cpp
│   │   ├── get_region_waypoint.cpp
│   │   ├── guidance.cpp
│   │   ├── hover.cpp
│   │   ├── list_recorded_targets.cpp
│   │   ├── play_audio.cpp
│   │   ├── re_heading.cpp
│   │   ├── recorded_targets.cpp
│   │   ├── round.cpp
│   │   ├── send_geo_pose_node.cpp
│   │   ├── takeoff.cpp
│   │   ├── target_check.cpp
│   │   ├── touch_down.cpp
│   │   ├── traverse_path_generator.cpp
│   │   ├── ugv_cowork_node.cpp
│   │   └── waitstep.cpp
│   ├── sim/
│   │   └── moving_target_plugin.cpp
│   ├── audio_caller.cpp
│   └── maxt_core.cpp
├── srv/                                         # ROS 服务接口定义
│   └── PlayAudio.srv                            # 音频播放服务的请求与响应格式
├── test/                                        # 任务行为树、轨迹算法和脚本测试
│   ├── __pycache__/                             # Python 自动生成的字节码缓存
│   │   └── test_fast_mission.cpython-38.pyc
│   ├── unit/                                    # 标定工具与轨迹曲线的单元测试
│   │   ├── test_calibrate_tools.cpp
│   │   ├── test_global_nav_curve.cpp
│   │   └── test_stop_curve.cpp
│   ├── nav_breakpoint_tree_test.cpp
│   ├── readme-test.md
│   ├── rollback_target_tree_test.cpp
│   ├── test_fast_mission.py
│   └── ugv_cowork_tree_test.cpp
├── worlds/                                      # Gazebo 场景，包含比赛场地、障碍物和跟踪测试环境
│   ├── competition.world
│   ├── obstacle_world.world
│   └── tracking_test.world
├── CMakeLists.txt                               # 配置 catkin 构建目标及依赖
├── README.md                                    # 项目说明
├── log.md                                       # 项目日志
├── note.md                                      # 开发笔记
└── package.xml                                  # ROS 包元数据与依赖声明
```
