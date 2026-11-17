# QR 目标异步发送到 UGV

当前入口使用公共 C++ `ag_client_cpp`，不添加代理节点，不修改飞行树或飞行设点逻辑。

对应 AG_Client 依赖提交：`a02edfb`（`feat(air-ground): add nonblocking C++ target dispatch and lifecycle logs`）。两仓库分别管理版本，构建时须同时提供此SDK版本与maxt接入版本。

`QrSurveySetup` 的原 QR `set_task` 被受理后，登记本次 mission_id 和 position_target_id。`QrSurveyContext` 收到该任务结果时，`QrUgvDispatch` 只选择与 position_target_id 一致的投放目标（当前配置 QR001）；必须 found、position_valid、revision>0，且坐标标签为 wgs84、测量时间非零、纬经度有限合法。普通 QR002/QR003、旧任务、未定位/仅解码结果不会派车。

目标首次有效时即入队，无须等扫描结束或 QrFinish。相同 mission/target 后续心跳与修订不再创建新的替换任务。只用视觉地理定位给出的地面经纬度，不用飞机位置、不传飞行高度、不重复做天线杆臂补偿。

```text
QR任务结果回调 → 选定目标/有效性检查 → AG_Client 入队
                                           │ 私有回调队列
BT 正常继续扫描/导航                         ├→ 非锁存 SET_TASK
                                           └← 受理/执行/完成观察与日志
```

## 开关与参数

默认开启，无需改 XML。由 maxt 私有参数控制：

```yaml
ugv_dispatch:
  enabled: true
  state_timeout: 5.0
  connect_timeout: 3.0
  receipt_timeout: 3.0
  state_max_age: 2.0
  execution_max_age: 2.0
```

离线纯飞行回归或暂不希望派车时设 `ugv_dispatch/enabled=false`。实际运行仍需两端同版消息、ROS 双向网络/时钟、车端任务与导航就绪；本功能不会启动 MAVROS、起飞、解锁或解除 UGV hold。

UGV 不可用只影响派车日志，不使飞行 BT 等待或失败。发送后回执超时不代表未执行，不自动重发或取消；同一次入队失败也不会在每个 QR 心跳上创建新请求。需要重新派车须由人员查看状态后通过唯一命令出口明确处理。

日志统一前缀 `[AG_Client]`：已发送二维码与实际经纬度、UGV已受理、同任务开始执行、最终证据完整后任务完成；阻塞/拒绝/未知结果单独输出。正常 ROS log/rosout 保留这些记录。

## 无飞行验证

新增 `qr_ugv_dispatch_probe` 只构造真实 QR context、C++ 客户端和无副作用 BT 心跳，不构造 MavKit，不运行生产飞行入口。测试使用独立 loopback master、合成二维码和假 UGV；不启动 MAVROS、飞控、相机、底盘或 Takeoff 节点。

```bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11379 ROS_IP=127.0.0.1
unset ROS_HOSTNAME
rostest maxt_pkg qr_ugv_dispatch.test
```

测试验证目标选择/坐标与时间门禁、心跳去重、首次发送与新任务身份、非锁存、跨 Python/C++ 发送锁、日志、受理不冒充执行、证据不完整不报完成、超时/重启不重发，以及等待期间 BT 心跳持续推进。该测试不证明实车/电台/测量精度。

2026-11-18：完整编译通过；18项上述无飞行集成测试全部通过，0错误/0失败；AG_Client既有52项测试全部通过。测试XML位于 `/tmp/maxt-ugv-test-results/maxt_pkg/`。中文日志已验证通过，maxt入口在ROS初始化前仅设置UTF-8字符分类locale，数字格式不变。用户原有未提交的飞行XML/参数保持原样，不纳入此次提交；未运行生产任务或更新其install配置。
