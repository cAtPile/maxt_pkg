#include <maxt_pkg/maxt_nodes/goto_node.hpp>
#include <cmath>

namespace maxt {

BT::PortsList GoToNode::providedPorts() {
    return {
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("z"),
        BT::InputPort<double>("timeout", 30.0, "timeout (s)"),
        BT::InputPort<double>("tolerance", 0.3, "arrival tolerance (m)"),
        BT::InputPort<double>("step", 0.5, "virtual target step (m)")
    };
}

GoToNode::GoToNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav)
{
}

BT::NodeStatus GoToNode::onStart() {
    if (!getInput<double>("x", tx_) ||
        !getInput<double>("y", ty_) ||
        !getInput<double>("z", tz_)) {
        ROS_ERROR("GoToNode: Missing target coordinates!");
        return BT::NodeStatus::FAILURE;
    }

    hold_yaw_ = mav_.get_current_yaw();
    getInput<double>("step", step_);
    getInput<double>("timeout", timeout_);
    getInput<double>("tolerance", tolerance_);
    start_time_ = ros::Time::now();

    ROS_INFO("GoToNode: Target [%.2f, %.2f, %.2f] yaw=%.2f step=%.2f",
             tx_, ty_, tz_, hold_yaw_, step_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToNode::onRunning() {
    ros::Time now = ros::Time::now();

    if ((now - start_time_).toSec() > timeout_) {
        ROS_ERROR("GoToNode: Timeout!");
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::FAILURE;
    }

    auto current = mav_.getCurrentPose();
    double cx = current.pose.position.x;
    double cy = current.pose.position.y;
    double cz = current.pose.position.z;

    if (std::fabs(tx_ - cx) < tolerance_ &&
        std::fabs(ty_ - cy) < tolerance_ &&
        std::fabs(tz_ - cz) < tolerance_) {
        ROS_INFO("GoToNode: Target reached!");
        mav_.setTargetPose(tx_, ty_, tz_);
        mav_.setSetpointMode(SetpointMode::CONTROL);
        return BT::NodeStatus::SUCCESS;
    }

    double dx = tx_ - cx;
    double dy = ty_ - cy;
    double dz = tz_ - cz;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    double spx, spy, spz;
    if (dist <= step_) {
        spx = tx_;
        spy = ty_;
        spz = tz_;
    } else {
        spx = cx + (dx / dist) * step_;
        spy = cy + (dy / dist) * step_;
        spz = cz + (dz / dist) * step_;
    }

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
    target.position.x = spx;
    target.position.y = spy;
    target.position.z = spz;
    target.yaw = hold_yaw_;
    mav_.setRawTarget(target);
    mav_.setSetpointMode(SetpointMode::RAW_CTRL);

    return BT::NodeStatus::RUNNING;
}

void GoToNode::onHalted() {
    ROS_WARN("GoToNode: Halted! Holding position.");
    mav_.setSetpointMode(SetpointMode::HEARTBEAT);
}

} // namespace maxt
