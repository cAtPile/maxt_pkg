# cowork-infoStream.md

空地协同信息流

## ReadyCheck.srv

用于确定UGV是否可以执行任务

```
# ReadyCheck.srv

bool ready # 1 ready 0 not ready
UgvReadyState not_ready_code # reason not ready

UgvReadyState{
    MISSION_RUNNING, //任务没有完成，等待任务完成
    UGV_STATE_UNHEALTH, //无人车状态不健康，报错并退出
    UGV_RETRY //无人车自我修复中等待一段时间后，进行任务
    //... 其他具体错误原因
    UNKNOWN_NOT_READY, //未知原因没能就绪，报错并退出
}
```

## MissionTarget.action

用于交互任务
```
# MissionTarget.action

# goal
uint8 target_id #目标id
PoseType target_pose_type #目标位置信息格式
PoseType{
    LOCAL_TYPE,//相对坐标
    GOLBAL_TYPE//经纬度
}
geometry_msgs::PoseStamped target_pose #目标位置
UgvAction target_action #目标点行动
UgvAction{
    PASS_BY,//经过目标点
    DELIVER,//投放
    AVOIDANCE,//避障
    //... 其他行动
    UNDESIGNED_ACTION//未定义的任务
}
---
# result
bool succeed #1成功 0失败
UgvMissionErrorReason error_code
UgvMissionErrorReason{
    DESTRUCTIVE_ERROR,//毁灭性失败，终止任务
    TIMEOUT_ERROR,//超时失败，重新尝试一次
    LOST_TARGET_ERROR,//失去目标，重新尝试
    UNKNOWN_ERROR,//未知失败，终止任务
}
---
# feedback
UgvMissionState ugv_feedback_state
UgvMissionState{
    WAITING,//等待任务
    MOVING,//正在移动
    DELIVERING,//正在投放
    //...其他状态
    UNDESIGNED_STATE,//未定义的状态
}

```