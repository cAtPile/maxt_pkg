# craic 相关内容清理交付

## 修改内容与设计

按 craicRM_guide.md 清理比赛专用能力，复用基础飞行节点与 MavKit，保持其实现不变。移除专用模块时同时清理头文件、实现、工厂注册、构建与启动引用，避免核心仍依赖已移除节点。

移除 AllDeliver、Deliver、QRDetect、RingCenterReceiver、TargetCheck2、YoloReceiver 的头文件和实现，以及独立圆环中心检测程序。同步移除无实现的 RingTarget/RingPass 历史声明、QR/YOLO 脚本、模型权重、投放仿真和测试脚本、圆环回显与绘图脚本、圆环历史图片。

清理原有 config 测试 XML、launch/example 和 launch/sim 启动文件。新增 config/mission/basic_test.xml 和 launch/basic_test.launch，流程为连接检查、起飞 1m、悬停 3s、降落。通用 mission.launch 保留基础飞行入口及 MAVROS、FAST-LIO、通用相机开关。

删除仅服务于识别模块的 StringStamped.msg 和消息生成配置，以及圆环检测专用的 PCL、点云转换、可视化依赖。保留通用相机脚本及其 sensor_msgs 运行依赖，保留通用飞行监视工具；追加清理 conditions 条件节点。README 已更新为当前功能和启动方式。

## 接口变更

- BT 工厂移除 AllDeliver、Deliver、QRDetect、RingCenterReceiver、TargetCheck2、YoloReceiver；原 XML 调用这些节点需改写。
- 移除 maxt_ring_center_detect_node 可执行目标和 maxt_pkg/StringStamped 消息类型。
- 本包不再提供 qr_detect_result、yolo_detect、/ring_center 检测输出，以及 YOLO 脚本提供的检测控制接口。
- 核心不再发布 /servo/{front_left,front_right,back_left,back_right,all}/{open,close}，不再初始化 is_delivered_* 黑板键。
- mission.launch 移除 run_ring_detect、run_qr_detect、run_yolo_detect、run_real_deliver、run_fake_deliver 及圆环 ROI、YOLO 参数；bt_xml_path 改为必填参数。
- 移除 conditions 的头文件、实现、构建项及 CheckLandLeft/CheckLandRight 工厂注册；不再提供读取 land_target 的条件节点。基础飞行节点和 MAVROS 接口保持原实现。

## 新测试套件

追加 navigation_test.xml / navigation_test.launch：使用 QuinticNav 在高度 1m 依次经过 (0,0)、(2,0)、(2,2)，再返回 (0,0) 降落，航点间悬停 2s。限速 0.8m/s、限加速度 0.5m/s²。

追加 round_test.xml / round_test.launch：先航行至圆周起点 (0,0,1)，以 (1,0,1) 为圆心、半径 1m，顺时针转 360°，角速度 20°/s、加减速时间 2s，随后返回原点降落。先显式到达圆周起点，避免圆周节点启动时因位置与指定半径不同发生目标跳变。

两个测试复用已有 mission.launch 和飞行节点；使用绝对坐标，假定起飞点 XY 为 (0,0)、目标高度 z 为 1m，不自动标定。不同定位原点需要调整配置。QuinticNav 末端等待超时后会返回 SUCCESS，验收时须结合实际轨迹判定精度。

## 检验方式

执行 `roslaunch maxt_pkg basic_test.launch`，使用已启动的 MAVROS 和定位系统。需要自动启动外部组件时，使用 README 所列开关。任务包含实际起飞和降落，配置参数可直接在各测试 XML 中调整。新增测试分别执行 `roslaunch maxt_pkg navigation_test.launch` 和 `roslaunch maxt_pkg round_test.launch`。

按指南要求未执行编译、ROS 运行或飞行测试；当前修改仅完成源码及引用审阅，实际运行结果待用户检验。

## 三条核心建议

1. 清理后重新构建工作空间并重新加载环境，避免旧二进制或生成消息造成接口仍存在的假象。
2. 先通过简单起飞、悬停、降落检查基础闭环，再逐步加入 GoTo、QuinticNav 和 Round。
3. 后续新任务复用 mission.launch 和基础节点，将新识别或机构能力独立组织，减少任务专用代码对核心的耦合。
