# cowork-infoStream.md

空地协同信息流

## 消息流程
### 正常消息流动
uav查询ugv当前状态 通过/ugv/readiness_check，消息类型CheckReainess.srv,ugv返回 UGV_MISSION_READY 的信号
uav接收 UGV_MISSION_READY 信号后，通过action发布 targrt_id target_pose target_t