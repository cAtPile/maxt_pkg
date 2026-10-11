# Guidance 固定误差测试

## 修改及接口

修改 `script/guidance_test_server.py`，新增 `config/mission/guidance_test.xml` 和 `launch/guidance_test.launch`，更新 README。
Action 类型及 Guidance 节点接口不变。测试服务仅接受 `target_id=test`，其他目标返回 ABORTED。
脚本私有参数为 `error_u=20`、`duration=10`、`rate=20`；移除旧 error_v 参数和误差线性收敛行为。
launch 提供同名参数及 `run_sim=false`、`gui=true`。

## 启动

编译最新节点和消息后加载工作空间环境：

```bash
cd ~/catkin_ws
catkin_make --pkg maxt_pkg
source devel/setup.bash
```

已有 sim.launch 仿真时：

```bash
roslaunch maxt_pkg guidance_test.launch
```

一起启动仿真时：

```bash
roslaunch maxt_pkg guidance_test.launch run_sim:=true
```

只启动反馈服务时：

```bash
rosrun maxt_pkg guidance_test_server.py
```

## 设计与预期行为

行为树依次连接检查、起飞到 1m、悬停 2 秒、Guidance 测试、降落。
收到 test goal 后，以 20Hz 发布 target_visible=true、error_u=20、error_v=0。
从收到 goal 起经过 10 秒后，不再发布任何 feedback，但 Action 保持 ACTIVE，直到客户端取消。
采用 ROS 时间，仿真暂停时计时也暂停。反馈为固定测试值，不反映真实图像或飞机位置。

误差大于容差 5，节点不会达标成功。反馈停止约 0.5 秒后，当前 Guidance 实现从 RAW_CTRL 切换 HEARTBEAT 并进入 WAIT_FEEDBACK。
整体 timeout=15 秒从 Guidance 启动计时，连接等待也包含其中；最终打印 TIMEOUT 并取消 goal。
ForceSuccess 装饰器将预期的 Guidance 失败转为成功，使序列继续降落；Guidance 的 message 仍是 TIMEOUT。
这不是整个引导任务成功的判定，而是超时测试的清理流程。
默认 u 轴映射到世界 x，正误差产生负向速度，kp=0.01 时 error_u=20 对应约 -0.2m/s。

## 验证

检查 Python 语法、XML 解析和补丁格式；未运行 PX4/Gazebo 完整飞行。

## 三条核心建议

1. 保持 error_u 大于 tolerance，以保证反馈停止前不会提前达标。
2. duration 小于行为树 timeout，留出反馈过期和等待观察时间。
3. 不要同时启动两个 /guidance 服务；先在仿真中确认坐标方向，再接入真实反馈。
