#include <maxt_pkg/maxt_nodes/hover_node.hpp>

namespace maxt {

BT::PortsList HoverNode::providedPorts() {
    return {
        BT::InputPort<double>("hover_duration", 1.0, "hover duration (s)")
    };
}

HoverNode::HoverNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav) {}

BT::NodeStatus HoverNode::onStart() {
    if (!getInput<double>("hover_duration", hover_duration_)) {
        hover_duration_ = 1.0;
    }

    start_hover_pose_ = mav_.GetRawPose();
    start_time_ = ros::Time::now();

    const auto& position = start_hover_pose_.pose.position;
    mav_.setTargetPose(position.x, position.y, position.z);
    mav_.setSetpointMode(SetpointMode::RAW_CTRL);

    ROS_INFO("HoverNode: holding at (%.2f, %.2f, %.2f) for %.2f s",
             position.x, position.y, position.z, hover_duration_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoverNode::onRunning() {
    if ((ros::Time::now() - start_time_).toSec() >= hover_duration_) {
        ROS_INFO("HoverNode: hover completed.");
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }

    // 每个 tick 持续更新记录的位置目标
    const auto& position = start_hover_pose_.pose.position;
    mav_.setTargetPose(position.x, position.y, position.z);

    return BT::NodeStatus::RUNNING;
}

void HoverNode::onHalted() {
    ROS_WARN("HoverNode: halted!");
    mav_.setSetpointMode(SetpointMode::HEARTBEAT);
}

} // namespace maxt
