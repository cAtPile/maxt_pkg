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

namespace {
constexpr char kPx4HoldMode[] = "AUTO.LOITER";
}

BT::NodeStatus Px4HoldNode::onStart() {
    started_ = ros::SteadyTime::now();
    next_request_ = ros::SteadyTime();
    return onRunning();
}

BT::NodeStatus Px4HoldNode::onRunning() {
    const auto mode = mav_.getMode();
    if (mav_.isConnected() && mav_.isArmed() && mode == kPx4HoldMode) {
        mav_.setSetpointMode(SetpointMode::STANDBY);
        ROS_INFO("%s: PX4 Hold confirmed (%s).", name().c_str(), kPx4HoldMode);
        return BT::NodeStatus::SUCCESS;
    }
    if (mode != "OFFBOARD" || !mav_.isArmed() || !mav_.hasFreshNavigationData()) {
        mav_.setSetpointMode(SetpointMode::STANDBY);
        ROS_WARN("%s: Hold handoff interrupted. Mode: %s.", name().c_str(), mode.c_str());
        return BT::NodeStatus::FAILURE;
    }
    const auto now = ros::SteadyTime::now();
    if (next_request_.isZero()) {
        using Target = mavros_msgs::PositionTarget;
        Target target;
        target.coordinate_frame = Target::FRAME_LOCAL_NED;
        target.type_mask = Target::IGNORE_VX | Target::IGNORE_VY | Target::IGNORE_VZ |
                           Target::IGNORE_AFX | Target::IGNORE_AFY | Target::IGNORE_AFZ |
                           Target::IGNORE_YAW_RATE;
        target.position = mav_.GetRawPose().pose.position;
        target.yaw = mav_.get_current_yaw();
        mav_.setRawTarget(target, false);
        mav_.setSetpointMode(SetpointMode::RAW_CTRL);
        next_request_ = now;
    }
    if ((now - started_).toSec() >= 5.0) {
        ROS_ERROR_THROTTLE(5.0, "%s: PX4 Hold unconfirmed. Holding position in OFFBOARD.",
                           name().c_str());
    } else if (now >= next_request_) {
        next_request_ = now + ros::WallDuration(1.0);
        mav_.requestMode(kPx4HoldMode);
    }
    return BT::NodeStatus::RUNNING;
}

void Px4HoldNode::onHalted() {
    mav_.setSetpointMode(SetpointMode::STANDBY);
}

} // namespace maxt
