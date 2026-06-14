#include <maxt_pkg/maxt_nodes/waitstep_node.hpp>

namespace maxt {

BT::PortsList WaitStepNode::providedPorts() {
    return {
        BT::InputPort<double>("wait_duration",1.0, "等待时间")
    };
}

WaitStepNode::WaitStepNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav) {
}

BT::NodeStatus WaitStepNode::onStart() {
    // 获取等待时间参数
    if (!getInput<double>("wait_duration", wait_duration_)) {
        wait_duration_ = 1.0; // 默认等待1秒
    }

    start_time_ = ros::Time::now();
    ROS_INFO("WaitStepNode: Waiting for %.2f seconds...", wait_duration_);
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitStepNode::onRunning() {
    // 检查是否达到等待时间
    if ((ros::Time::now() - start_time_).toSec() >= wait_duration_) {
        ROS_INFO("WaitStepNode: Wait completed.");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void WaitStepNode::onHalted() {
    ROS_WARN("WaitStepNode: Halted during wait operation!");
    // BT::StatefulActionNode::halt();  // 基类 halt() 已调用 onHalted()，子类不应反向调用
}

} // namespace maxt