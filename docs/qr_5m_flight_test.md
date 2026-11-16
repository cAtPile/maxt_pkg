# 起飞机头坐标系停点巡检

配置文件：`config/qr_stop_survey.yaml`。本次保留现场配置：对角点 `(10,10)` 米、
步长 5 米、巡检高度 2.5 米，合计 9 个停点。默认普通码 QR002、QR003，投放码 QR001。

## 范围、原点和方向

检测到外部 ARM 时，记录当时的原始 local yaw，整个巡检固定使用该方向：
**+X 为机头前方、+Y 为飞机左侧、-Y 为飞机右侧**。等待 ARM 期间转动机头，使用转动后
实际解锁时的方向；飞行途中 yaw 变化不会旋转已定义的区域。原点 XY 在进入爬升前记录，
高度使用本次起飞的目标 Z，不把上电/local 原点当作区域角点。该巡检树不再使用
`nav/azimuth_angle` 定义区域方向；其他任务树仍保留其原有方向行为。

`(0,0)` 和 `(corner_x,corner_y)` 是矩形对角点。例如：

| corner_x | corner_y | 覆盖方向 |
|---|---|---|
| 5 | -5 | 前方 5 米、右侧 5 米 |
| 5 | 5 | 前方 5 米、左侧 5 米 |
| -5 | -5 | 后方 5 米、右侧 5 米 |
| -5 | 5 | 后方 5 米、左侧 5 米 |

正负号定义方向，两个坐标都必须非零。长度是两坐标绝对值，步长必须为正；步长无法整除
边长时仍包含终点，最后一段会短一些。为兼容旧配置，未提供任何 `corner_*` 时仍可读取
`length_x/length_y`，但其参考方向同样采用起飞机头方向。建议迁移到明确的对角点参数。

例如前右侧 5×5 米、间隔 1 米：

```yaml
nav:
  v_max: 0.5  # m/s
  a_max: 0.5  # m/s^2
qr_survey:
  # 保留文件中的二维码 ID、时间限制等其余配置
  corner_x: 5.0
  corner_y: -5.0
  step: 1.0
  survey_height: 2.5
```

顺序为先沿 Y 走一列、再沿 X 换列，蛇形往返：
`(0,0) → (0,-1) → … → (0,-5) → (1,-5) → … → (1,0) → (2,0) → …`，共 36 个停点。
若画图时机头向上、飞机右侧向右，`(5,-5)` 对应飞机位于区域左下角。
范围是航点中心范围，不是桨尖范围或地理围栏；任务不含避障或投放坐标的区域围栏。

## 速度配置

在同一个 YAML 的 `nav/v_max` 和 `nav/a_max` 修改速度和加速度，无需编译。
`overall.sh start_survey:=true` 与独立的 `qr_stop_survey.launch` 均默认使用 YAML。
仅明确传入 `survey_speed:=...`、`survey_acceleration:=...` 时才覆盖对应项目。
实际曲线速度受航段长度和停点要求约束，不保证达到上限。修改后在地面重启巡检任务生效。

每点确认低速/低倾角/位置稳定 0.4 秒，再延时 1 秒，选取一张新的图像和对应新深度。
即使提前找到全部码，也会走完整个配置网格。结束后保持 RUNNING 等待，不自动投放、返航或降落。

## 先在地面准备

1. 落地、上锁，关闭旧任务/旧视觉进程后加载新版本。不得在空中重新启动 Takeoff 任务，
   因为其高度是从新一轮起飞位置建立的。不能同时运行两个飞行任务执行器。
2. 核对相机确实按 geolocation.yaml 下视安装，图像上方对应机头，主天线是 ant1，外参尺寸正确。
   不要同时运行发布同名安装 TF 的 uav_tf_test。
3. 使用已验证的 MAVROS 连接方式，仅运行一套；本节的任务入口均不启动 MAVROS。
   检查定位、速度、相对高度、原始 RTK 和 IMU 连续更新，地面相对高度接近零且 Home 正确。
4. 在地面站确认 RTK 定位质量、航向、飞控健康、遥控模式接管和失联保护。任务的
   `geolocation_input_status=VALID` 仅表示输入数值/时间有效，不能单独证明 RTK fixed 或厘米级精度。
5. 先拆桨验证视觉及屏幕接口，之后按现场流程恢复桨叶并清场。

每个终端加载同一环境；env.sh 中的 ROS_IP/ROS_MASTER_URI 必须与现场地址一致：

```bash
source /home/dlab/qr_visual_release_20261114_Q1tLWg/env.sh
source /home/dlab/catkin_ws/devel/setup.bash
```

若尚未运行视觉系统，在单独终端启动（复用已有 MAVROS/ROS Master）：

```bash
roslaunch /home/dlab/qr_visual_release_20261114_Q1tLWg/qr_ground.launch
```

该文件包含 ODIN、OYO、二维码视觉及任务节点，不能与旧的同名实例重复启动。
关键地面检查：

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /uav_qr_mission/result/geolocation_input_status
rosservice type /ugv/qr_display/update
```

预期 connected=true、地面 armed=false，定位输入状态为 VALID。
小车服务应为 `uav_qr_mission/ReportQr`，不是任意旧服务；需先按二维码包的
`docs/ugv_display_service.md` 适配并完成 FKIE 服务同步。

### 地面视觉和屏幕验收

确保 maxt 飞行执行器没有运行，然后登记地面检查任务：

```bash
rosservice call /uav_qr_mission/set_task \
  "{mission_id: 'ground_check_qr001_01', target_ids: ['QR002','QR003','QR001'], position_target_id: 'QR001', capture_only: false}"
roslaunch uav_qr_mission qr_report_sender.launch service:=/ugv/qr_display/update
```

逐个展示码、再同时展示多个码。查看 `/uav_qr_mission/result` 的 targets，确认内容、位置有效标志、
测量时间及是否投放码；用现场独立测量核对定位方向和位置，不能只看 valid。
小车应单条出现、批量更新、重复报告不增加重复行。另一个终端看 `/qr_report_sender/status`。
每个码首次定位后会保存，移动码重测应换一个新的 mission_id。

地面检查结束后停止这个单独启动的 sender，正式任务会自行启动一个；或正式启动时设置
`start_report_sender:=false` 复用它。正式任务自动注册新的任务 ID，不需要手动重复注册。

## 分阶段试飞

| 阶段 | 项目 | 通过条件 |
|---|---|---|
| A | 起飞 3 m、原地保持、遥控接管降落 | 高度和速度稳定才进入 TakeoffTestHold，无水平导航，能人工接管 |
| B | 1×1 m、步长 1 m 的四点小格 | 方向正确、各航段真实到达、停稳延时后才采集 |
| C | 5×5 m 完整 36 点扫描 | 不提前结束；注册内容、位置、测量时间正确并到达小车 |
| D | 扫描后到投放码上空 3 m 待命 | 位置符合独立标定结果，进入 ReadyForDrop 并持续保持 |

A 项命令：飞机在地面，保持未解锁，启动后先确认等待外部解锁；一旦外部解锁将自动请求 OFFBOARD 并起飞。

```bash
roslaunch maxt_pkg mission.launch sim:=false run_uav:=false \
  bt_xml_path:=/home/dlab/catkin_ws/src/maxt_pkg/config/mission/qr_takeoff_hold_test.xml
```

通过后人工接管、降落、上锁，停止该任务，再进行下一项。不要在空中直接切换这些任务。

B 项先复制配置，不修改正式配置：

```bash
python3 - <<'PY'
import yaml
p='/home/dlab/catkin_ws/src/maxt_pkg/config/qr_stop_survey.yaml'
c=yaml.safe_load(open(p))
c['qr_survey']['corner_x']=1.0
c['qr_survey']['corner_y']=-1.0
c['qr_survey']['step']=1.0
with open('/tmp/qr_1m_survey.yaml','w') as f:
    yaml.safe_dump(c,f)
PY
roslaunch maxt_pkg qr_stop_survey.launch \
  survey_config:=/tmp/qr_1m_survey.yaml \
  survey_speed:=0.5 survey_acceleration:=0.5
```

B 项将投放码布置在这个 1×1 m 小格内，否则最后的经纬度导航仍可能前往小格之外的投放码。
二维码位置和配置只在降落、上锁后调整。

C/D 项完整任务：

```bash
roslaunch maxt_pkg qr_stop_survey.launch \
  survey_speed:=0.5 survey_acceleration:=0.5 \
  start_qr_mission:=false \
  display_service:=/ugv/qr_display/update
```

`start_qr_mission=false` 用于复用 qr_ground.launch 的任务节点。该任务同样从地面开始，
等连接和 RTK/IMU 就绪后停在外部解锁等待阶段；外部解锁后自动执行，不要提前人工起飞。
未解锁时运行 `rosparam get /maxt_test_node/qr_survey` 和 `rosparam get /maxt_test_node/nav`，
核对实际加载的对角点、步长、高度、速度与预期一致。当前保留配置应为 `(10,10)`、步长 5、
高度 2.5 米，日志显示 9 个停点；若改用前右侧 `(5,-5)`、步长 1，则应显示 36 个停点。
出现 `Waiting for external arming` 后，按现场流程确认再解锁。
普通码缺失会保留未定位状态；投放码有效时仍可转场。投放码无有效坐标、航点/采集失败则进入失败保持。

## 观察、记录与结束

```bash
rostopic echo /maxt_test_node/mission_status
rostopic echo /uav_qr_mission/result
rostopic echo /qr_report_sender/status
rostopic echo /mavros/global_position/rel_alt
```

地面试飞前可另开终端录制轻量数据，文件名每架次更换：

```bash
rosbag record -O /tmp/qr_5m_test_01.bag \
  /mavros/state /mavros/local_position/pose /mavros/local_position/velocity_local \
  /mavros/global_position/global /mavros/global_position/raw/fix \
  /mavros/global_position/rel_alt /mavros/imu/data \
  /uav_odin_qr/detections /uav_odin_qr/tf_targets \
  /uav_qr_mission/result /qr_report_sender/status /maxt_test_node/mission_status
```

扫描完成看 `survey_finished`，三个码定位看 `all_localized`，待命看 BT 的 `ReadyForDrop`。
旧 `complete` 含 arena 条件，不作为本轮扫图完成标准。空检测帧也可以是一次成功采集。
飞行中视觉输出在非采集阶段暂停是正常现象，任务节点仍有 2 Hz 心跳。
local Z 要看相对起飞前的增量；相对高度应接近配置的 survey_height，不能把 WGS84 椭球高当作飞行高度。

异常时先使用已验证的遥控模式开关接管，稳定后降落，再停止任务；不要用 Ctrl+C 当作空中刹车。
Offboard 需要持续设点，停掉程序可能触发飞控配置的失联动作。接管/失联保护的行为以实际固件和参数为准，
参考 [PX4 Offboard 官方说明](https://docs.px4.io/main/en/flight_modes/offboard)。
断开定位、制造超时等故障注入只在拆桨地面或模拟中进行，不作为首轮空中试飞项目。
