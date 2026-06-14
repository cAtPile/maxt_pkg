# Mission XML 编写指南

## 基本结构

```xml
<root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
        <!-- 行为树主体 -->
    </BehaviorTree>

    <TreeNodesModel>
        <!-- 所有节点的参数声明 -->
    </TreeNodesModel>
</root>
```

`main_tree_to_execute` 必须与 `<BehaviorTree>` 的 `ID` 匹配。

## 控制流节点

| 节点 | 行为 |
|------|------|
| `<Sequence>` | 顺序执行子节点，任一失败则整体失败 |
| `<Fallback>` | 依次尝试子节点，任一成功则整体成功 |
| `<Parallel M="N">` | 并行执行，M 个成功即成功 |

项目中最常用的是 `<Sequence>`，所有任务按顺序排列。

## 参数传递

参数以 XML 属性形式传入，名称必须与 `providedPorts()` 中的 `InputPort` 名完全一致：

```xml
<QuinticNav x="1.8" y="0.0" z="1.2" timeout="30" tolerance="0.1" v_max="2.0" a_max="1.0" />
```

## TreeNodesModel 声明

每个在树中使用的节点必须在 `<TreeNodesModel>` 中声明：

```xml
<TreeNodesModel>
    <!-- Action 节点 -->
    <Action ID="NodeName">
        <parameter name="param_name" type="double" default="0.0" />
        <parameter name="param_name2" type="string" default="" />
    </Action>

    <!-- Condition 节点 -->
    <Condition ID="NodeName" />
</TreeNodesModel>
```

参数类型支持: `double`, `string`, `int`

## 全部可用的 BT 节点及参数

### ConnectCheck
```xml
<Action ID="ConnectCheck">
    <parameter name="timeout" type="double" default="60.0" />
</Action>
```

### Takeoff
```xml
<Action ID="Takeoff">
    <parameter name="target_alt" type="double" default="1.5" />
    <parameter name="ascent_speed" type="double" default="0.5" />
    <parameter name="timeout" type="double" default="20.0" />
</Action>
```

### QuinticNav — 五次多项式轨迹导航
```xml
<Action ID="QuinticNav">
    <parameter name="x" type="double" />
    <parameter name="y" type="double" />
    <parameter name="z" type="double" />
    <parameter name="timeout" type="double" default="30.0" />
    <parameter name="tolerance" type="double" default="0.1" />
    <parameter name="v_max" type="double" default="2.5" />
    <parameter name="a_max" type="double" default="1.2" />
</Action>
```

### GoTo — 简单分段导航
```xml
<Action ID="GoTo">
    <parameter name="x" type="double" />
    <parameter name="y" type="double" />
    <parameter name="z" type="double" />
    <parameter name="timeout" type="double" default="30.0" />
    <parameter name="tolerance" type="double" default="0.2" />
    <parameter name="step" type="double" default="0.5" />
</Action>
```

### WaitStep — 等待
```xml
<Action ID="WaitStep">
    <parameter name="wait_duration" type="double" default="1.0" />
</Action>
```

### Round — 圆形盘旋
```xml
<Action ID="Round">
    <parameter name="center_x" type="double" />
    <parameter name="center_y" type="double" />
    <parameter name="center_z" type="double" />
    <parameter name="radius" type="double" default="0.0" />
    <parameter name="angular_speed" type="double" default="30.0" />
    <parameter name="total_angle" type="double" default="360.0" />
    <parameter name="tolerance" type="double" default="0.3" />
    <parameter name="timeout" type="double" default="60.0" />
</Action>
```

### QRDetect — QR 码检测
```xml
<Action ID="QRDetect">
    <parameter name="timeout" type="double" default="5.0" />
</Action>
```

### YoloReceiver — YOLO 目标识别
```xml
<Action ID="YoloReceiver">
    <parameter name="timeout" type="double" default="10.0" />
</Action>
```

### RingCenterReceiver — 圆环中心检测
```xml
<Action ID="RingCenterReceiver">
    <parameter name="timeout" type="double" default="10.0" />
    <parameter name="offset" type="double" default="0.7" />
    <parameter name="bias" type="double" default="0.1" />
</Action>
```

### Deliver — 单仓投放
```xml
<Action ID="Deliver">
    <parameter name="servo_a" type="string" default="front_left" />
    <parameter name="servo_b" type="string" default="front_right" />
    <parameter name="deliver_delay" type="double" default="2.0" />
    <parameter name="connect_delay" type="double" default="0.2" />
</Action>
```

servo 可选值: `front_left`, `front_right`, `back_left`, `back_right`
fallback 逻辑: 先尝试 `servo_a`，若已用则尝试 `servo_b`

### AllDeliver — 全部门投放
```xml
<Action ID="AllDeliver">
    <parameter name="deliver_delay" type="double" default="2.0" />
</Action>
```

### TouchDown — 降落
```xml
<Action ID="TouchDown">
    <parameter name="descent_speed" type="double" default="-0.5" />
    <parameter name="timeout" type="double" default="60.0" />
    <parameter name="ground_timeout" type="double" default="3.0" />
</Action>
```

### Condition 节点
```xml
<Condition ID="CheckLandLeft" />
<Condition ID="CheckLandRight" />
<Condition ID="TargetCheck2" />
```

`CheckLandLeft` / `CheckLandRight`: 检查 blackboard 中 `land_target` 的值
`TargetCheck2`: 比对 `current_object` (YOLO结果) 与 `target_1`/`target_2` (QR结果)

## 典型任务模板

```xml
<root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
        <Sequence>
            <!-- 1. 连接检查 -->
            <ConnectCheck timeout="30" />

            <!-- 2. 起飞 -->
            <Takeoff target_alt="1.3" ascent_speed="0.5" timeout="20" />

            <!-- 3. 导航到目标点 -->
            <QuinticNav x="3.0" y="0.0" z="1.2" timeout="30" tolerance="0.1" v_max="2.0" a_max="1.0" />

            <!-- 4. 等待稳定 -->
            <WaitStep wait_duration="0.75" />

            <!-- 5. 降落 -->
            <TouchDown descent_speed="-0.5" timeout="60" ground_timeout="3.0" />
        </Sequence>
    </BehaviorTree>

    <TreeNodesModel>
        <!-- 所有使用到的节点声明 -->
    </TreeNodesModel>
</root>
```
