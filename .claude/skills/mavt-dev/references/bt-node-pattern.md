# BT 节点实现详细模式

## Action 节点（MavActionNode 子类）

### 头文件模板

```cpp
#ifndef MAXT_NODES_XXX_HPP
#define MAXT_NODES_XXX_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class XxxNode : public MavActionNode {
public:
    XxxNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    enum class Phase : uint8_t { STEP1, STEP2, DONE } phase_{Phase::STEP1};
    // 输入参数（从 XML / blackboard 读取）
    double param1_{1.0};
    double timeout_{10.0};
    // 运行时状态
    ros::Time start_time_;
};

} // namespace maxt
#endif
```

### 实现文件模板

```cpp
#include <maxt_pkg/maxt_nodes/xxx_node.hpp>

namespace maxt {

BT::PortsList XxxNode::providedPorts() {
    return {
        BT::InputPort<double>("param1", 1.0, "description"),
        BT::InputPort<double>("timeout", 10.0, "timeout (s)")
    };
}

XxxNode::XxxNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav) {}

BT::NodeStatus XxxNode::onStart() {
    // 1. 读取 input port
    if (!getInput<double>("param1", param1_)) param1_ = 1.0;
    if (!getInput<double>("timeout", timeout_)) timeout_ = 10.0;

    // 2. 初始化状态
    phase_ = Phase::STEP1;
    start_time_ = ros::Time::now();

    ROS_INFO("XxxNode: started");
    return BT::NodeStatus::RUNNING;  // StatefulActionNode 永远返回 RUNNING
}

BT::NodeStatus XxxNode::onRunning() {
    // 超时检测
    if ((ros::Time::now() - start_time_).toSec() > timeout_) {
        ROS_ERROR("XxxNode: timeout!");
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);  // 清理
        return BT::NodeStatus::FAILURE;
    }

    switch (phase_) {
    case Phase::STEP1: {
        // 执行操作，检查条件
        if (/* 条件满足 */) {
            phase_ = Phase::STEP2;
        }
        return BT::NodeStatus::RUNNING;
    }

    case Phase::STEP2: {
        // ...
        phase_ = Phase::DONE;
        return BT::NodeStatus::RUNNING;  // 下一 tick 进入 DONE
    }

    case Phase::DONE:
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void XxxNode::onHalted() {
    ROS_WARN("XxxNode: halted!");
    mav_.setSetpointMode(SetpointMode::HEARTBEAT);
}

} // namespace maxt
```

## Condition 节点（BT::ConditionNode 子类）

```cpp
// xxx_node.hpp
class XxxCondition : public BT::ConditionNode {
public:
    XxxCondition(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// xxx_node.cpp
XxxCondition::XxxCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

BT::NodeStatus XxxCondition::tick() {
    // 检查条件，同步返回 SUCCESS 或 FAILURE
    // 可从 blackboard 读取: config().blackboard->get<T>("key")
    return BT::NodeStatus::SUCCESS;
}
```

## 注册方式对比

| 类型 | 注册代码 | 特点 |
|------|----------|------|
| 需要 MavKit 的 Action | `factory_.registerBuilder<N>("N", [this](auto& n, auto& c) { return std::make_unique<N>(n, c, *mav_); })` | 捕获 `this`，传 `*mav_` |
| 需要额外 ROS publisher 的 Action | 同上，但构造时额外传入 pub_ | 如 DeliverNode 传 8 个 servo pub |
| Condition | `factory_.registerBuilder<C>("C", [](auto& n, auto& c) { return std::make_unique<C>(n, c); })` | 不捕获 `this`，不传 MavKit |

## Blackboard 访问

### 在 Action 节点中读/写 blackboard

```cpp
// 读取 (通过 InputPort 声明，onStart 中取值)
BT::InputPort<std::string>("key");

// 写入 (通过 OutputPort 或直接操作)
config().blackboard->set<std::string>("key", value);
// 或声明为 OutputPort
BT::OutputPort<std::string>("key");
setOutput<std::string>("key", value);
```

### MaxtCore 中初始化 blackboard

```cpp
auto bb = tree_.rootBlackboard();
bb->set("is_delivered_front_left", false);
```

## 关键生命周期规则

1. **onStart()** → 永远返回 `RUNNING`，不要返回 `SUCCESS`/`FAILURE`
2. **onRunning()** → 返回 `RUNNING`（继续）、`SUCCESS`（完成）、`FAILURE`（失败）
3. **onHalted()** → 清理 setpoint（切 HEARTBEAT），不返回值
4. DONE phase 模式：设置 `phase_ = DONE` 后 `return RUNNING`，下一 tick 进入 `case DONE` 返回 `SUCCESS`（确保最后有机会清理）
5. 超时模式：在 `onRunning()` 开头统一检测，超时后先清理再返回 `FAILURE`
