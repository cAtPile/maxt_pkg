#include <maxt_pkg/bt_logic_test_nodes.hpp>

#include <ros/ros.h>

namespace maxt {

TestConditionNode::TestConditionNode(const std::string& name,
                                     const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

/** 提供条件结果和日志标签。 */
BT::PortsList TestConditionNode::providedPorts() {
    return {
        BT::InputPort<bool>("value", "Condition result"),
        BT::InputPort<std::string>("label", "condition", "Log label")
    };
}

/** 读取配置值，输出判断结果，并将其转换为 SUCCESS/FAILURE。 */
BT::NodeStatus TestConditionNode::tick() {
    const bool value = getInput<bool>("value").value();
    const std::string label = getInput<std::string>("label").value();
    ROS_INFO("[BT logic test][Condition] %s -> %s",
             label.c_str(), value ? "SUCCESS" : "FAILURE");
    return value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

TestTickConditionNode::TestTickConditionNode(const std::string& name,
                                             const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

/** 提供条件保持成功的 tick 次数和日志标签。 */
BT::PortsList TestTickConditionNode::providedPorts() {
    return {
        BT::InputPort<unsigned int>("success_ticks", 1, "Number of successful ticks"),
        BT::InputPort<std::string>("label", "tick condition", "Log label")
    };
}

/** 在计数阈值前返回成功，达到阈值后返回失败。 */
BT::NodeStatus TestTickConditionNode::tick() {
    const unsigned int success_ticks = getInput<unsigned int>("success_ticks").value();
    const std::string label = getInput<std::string>("label").value();
    ++tick_count_;
    const bool value = tick_count_ <= success_ticks;
    ROS_INFO("[BT logic test][Reactive condition] %s tick=%u -> %s",
             label.c_str(), tick_count_, value ? "SUCCESS" : "FAILURE");
    return value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

TestLogNode::TestLogNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

/** 提供要写入 ROS 日志的文本。 */
BT::PortsList TestLogNode::providedPorts() {
    return {BT::InputPort<std::string>("message", "TestLog executed", "Log message")};
}

/** 输出日志并立即返回成功。 */
BT::NodeStatus TestLogNode::tick() {
    const std::string message = getInput<std::string>("message").value();
    ROS_INFO("[BT logic test][Action] %s", message.c_str());
    return BT::NodeStatus::SUCCESS;
}

TestRunningNode::TestRunningNode(const std::string& name,
                                 const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

/** 提供持续 tick 数和日志标签。 */
BT::PortsList TestRunningNode::providedPorts() {
    return {
        BT::InputPort<unsigned int>("running_ticks", 1, "Ticks before SUCCESS"),
        BT::InputPort<std::string>("label", "running action", "Log label")
    };
}

/** 初始化一次运行并返回 RUNNING，除非配置为零 tick。 */
BT::NodeStatus TestRunningNode::onStart() {
    running_ticks_ = getInput<unsigned int>("running_ticks").value();
    label_ = getInput<std::string>("label").value();
    tick_count_ = 0;
    ROS_INFO("[BT logic test][Running] %s started", label_.c_str());
    return running_ticks_ == 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

/** 增加运行计数，达到配置次数后成功。 */
BT::NodeStatus TestRunningNode::onRunning() {
    ++tick_count_;
    ROS_INFO("[BT logic test][Running] %s tick=%u/%u",
             label_.c_str(), tick_count_, running_ticks_);
    return tick_count_ >= running_ticks_
        ? BT::NodeStatus::SUCCESS
        : BT::NodeStatus::RUNNING;
}

/** 分支切换导致节点停止时输出 halt 日志。 */
void TestRunningNode::onHalted() {
    ROS_INFO("[BT logic test][Running] %s HALTED by control flow", label_.c_str());
}

TestFlakyNode::TestFlakyNode(const std::string& name,
                             const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

/** 提供成功前需要模拟的失败次数和日志标签。 */
BT::PortsList TestFlakyNode::providedPorts() {
    return {
        BT::InputPort<unsigned int>("failures_before_success", 1, "Failures before success"),
        BT::InputPort<std::string>("label", "flaky action", "Log label")
    };
}

/** 按调用次数返回模拟失败或成功，便于观察 Retry 的重试过程。 */
BT::NodeStatus TestFlakyNode::tick() {
    const unsigned int failures =
        getInput<unsigned int>("failures_before_success").value();
    const std::string label = getInput<std::string>("label").value();
    ++attempt_count_;
    const bool success = attempt_count_ > failures;
    ROS_INFO("[BT logic test][Retry action] %s attempt=%u -> %s",
             label.c_str(), attempt_count_, success ? "SUCCESS" : "FAILURE");
    return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace maxt
