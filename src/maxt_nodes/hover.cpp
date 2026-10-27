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

    start_hover_pose_ = mav_.getCurrentPose();
    start_hover_yaw_ = mav_.get_current_yaw();
    start_time_ = ros::Time::now();

    // 立即发布第一个 setpoint
    mavros_msgs::PositionTarget target;
    target.header.frame_id = "map";
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.type_mask =
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    target.position.x = start_hover_pose_.pose.position.x;
    target.position.y = start_hover_pose_.pose.position.y;
    target.position.z = start_hover_pose_.pose.position.z;
    target.yaw = start_hover_yaw_;
    mav_.setRawTarget(target);
    mav_.setSetpointMode(SetpointMode::RAW_CTRL);

    ROS_INFO("HoverNode: holding at (%.2f, %.2f, %.2f) for %.2f s",
             target.position.x, target.position.y, target.position.z,
             hover_duration_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoverNode::onRunning() {
    if ((ros::Time::now() - start_time_).toSec() >= hover_duration_) {
        ROS_INFO("HoverNode: hover completed.");
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }

    // 每个 tick 持续发布记录的位置
    mavros_msgs::PositionTarget target;
    target.header.frame_id = "map";
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.type_mask =
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    target.position.x = start_hover_pose_.pose.position.x;
    target.position.y = start_hover_pose_.pose.position.y;
    target.position.z = start_hover_pose_.pose.position.z;
    target.yaw = start_hover_yaw_;
    mav_.setRawTarget(target);

    return BT::NodeStatus::RUNNING;
}

void HoverNode::onHalted() {
    ROS_WARN("HoverNode: halted!");
    mav_.setSetpointMode(SetpointMode::HEARTBEAT);
}

} // namespace maxt
