# QR 巡检结果通过 AG Client V2 发送

在 `config/qr_stop_survey.yaml` 的 `qr_survey` 中填写二维码解码后的原文：

```yaml
qr_survey:
  target_id: QR001
  report_id: [QR002, QR003]
```

`target_id` 的首个有效地面 WGS84 坐标提交为 `SET_TARGET`。`report_id` 的两个码各自提交一次 `REPORT_QR`，携带二维码原文、经纬度、采样时间、位置版本和可用的椭球高。展示点可以先于导航目标送达。两种 V2 命令都携带同一测量帧的二维码居中 JPEG（2 倍放大），车端从本地 `/air_ground/v2/qr_capture` 接收已受理的图片。三条消息共用本轮巡检 ID 作为 V2 `task_id`。未注册的码不发送。

## 启动与检查

在同一个 UAV ROS master 上先启动 AG Client V2，车端运行 V2 的 `ag_ugv_task/ugv.launch`：

```bash
roslaunch ag_uav_client uav.launch
```

已有正确配置的 FKIE 使用 `start_fkie:=false` 启动 gateway。使用外部 `overall.sh` 时，巡检入口可加 `start_report_sender:=false` 关闭旧展示服务客户端。

maxt 在 gateway 已订阅且收到新鲜 UGV 状态后，向本机 `/air_ground/v2/uav_intent` 非锁存提交。默认最大状态年龄为 2 秒，两端应同步时钟。待提交期间飞行行为树继续运行。提交后由 V2 gateway 发送，maxt 不自动重发。将 maxt 私有参数 `ugv_dispatch/enabled` 设为 `false` 可关闭发送。

日志 `已向 V2 gateway 提交` 表示提交到本机。车端受理、展示数据和导航进度见：

```bash
rostopic echo -n 1 /air_ground/v2/state
```

查看 `last_command`、`last_qr_report`、`qr_reports` 和 `phase`，并核对同一 `task_id`。当前二维码结果没有图片标识，V2 的可选 `image_id` 留空。

## 隔离验证

在独立 loopback ROS master 上运行 `rostest maxt_pkg qr_ugv_dispatch.test`。测试使用真实 maxt QR 回调与 V2 gateway、合成二维码结果和模拟 UGV，不启动飞控或底盘。


## 巡检失败后的动作

巡检失败时优先播放 `cn-fault.wav`，并在 OFFBOARD 下发送固定位置设点。若本次曝光超时且一直未收到配对图像（`TIMEOUT_WAIT_FRAME`），已有本轮确认的投放目标坐标，且飞控仍已解锁、处于 OFFBOARD 且导航数据新鲜，定点保持 1 秒后走 `NavGeo` 直达该目标，再进入等待。缺少任一条件时持续保持。到目标途中导航失败也会转回定点保持。
