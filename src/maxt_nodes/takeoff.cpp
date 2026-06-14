#include <maxt_pkg/maxt_nodes/takeoff_node.hpp>

namespace maxt {

BT::PortsList TakeoffNode::providedPorts() {
    return {
        BT::InputPort<double>("target_alt", 0.5, "takeoff altitude"),
        BT::InputPort<double>("ascent_speed", 0.5, "ascent speed (m/s)"),
        BT::InputPort<double>("timeout", 30.0, "takeoff timeout(s)")
    };
}

TakeoffNode::TakeoffNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav) {
}

BT::NodeStatus TakeoffNode::onStart() {
    if (!getInput<double>("target_alt", target_alt_)) target_alt_ = 0.5;
    if (!getInput<double>("ascent_speed", ascent_speed_)) ascent_speed_ = 0.5;
    if (!getInput<double>("timeout", timeout_)) timeout_ = 30.0;

    phase_ = Phase::ARM;
    start_time_ = ros::Time::now();

    ROS_INFO("TakeoffNode: target_alt=%.2f m, ascent_speed=%.2f m/s", target_alt_, ascent_speed_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakeoffNode::onRunning() {

    ros::Time now = ros::Time::now();

    //起飞超时检测
    if ((now - start_time_).toSec() > timeout_) {
        ROS_ERROR("TakeoffNode: Timeout (%.1f s)!", timeout_);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::FAILURE;
    }

    switch (phase_) {

    //解锁
    case Phase::ARM: {
        if (!mav_.isArmed()) {
            mav_.requestArm(true);
            return BT::NodeStatus::RUNNING;
        }
        ROS_INFO("TakeoffNode: Armed, switching to OFFBOARD...");
        phase_ = Phase::OFFBOARD;
        return BT::NodeStatus::RUNNING;
    }

    //OFFBOARD & 获得速度
    case Phase::OFFBOARD: {
        if (mav_.getMode() != "OFFBOARD") {
            mav_.updateHomePose();
            mav_.setSetpointMode(SetpointMode::HEARTBEAT);
            mav_.requestMode("OFFBOARD");
            return BT::NodeStatus::RUNNING;
        }
        auto home = mav_.getHomePose();
        hold_x_ = home.pose.position.x;
        hold_y_ = home.pose.position.y;
        target_z_ = home.pose.position.z + target_alt_;
        ROS_INFO("TakeoffNode: OFFBOARD confirmed, home=(%.2f, %.2f, %.2f)",
                 hold_x_, hold_y_, home.pose.position.z);
        phase_ = Phase::INIT;
        return BT::NodeStatus::RUNNING;
    }

    //初始化
    case Phase::INIT: {
        auto current = mav_.getCurrentPose();

        mavros_msgs::PositionTarget target;
        target.header.frame_id = "map";
        target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        hold_yaw_ = mav_.get_current_yaw();

        target.type_mask =
            mavros_msgs::PositionTarget::IGNORE_PZ |
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        target.position.x = hold_x_;
        target.position.y = hold_y_;
        target.yaw = hold_yaw_;
        target.velocity.z = ascent_speed_;
        mav_.setRawTarget(target);
        mav_.setSetpointMode(SetpointMode::RAW_CTRL);

        ROS_INFO("TakeoffNode: Ascending from z=%.2f to z=%.2f at %.2f m/s, yaw=%.2f",
                 current.pose.position.z, target_z_, ascent_speed_, hold_yaw_);
        phase_ = Phase::ASCEND;
        return BT::NodeStatus::RUNNING;
    }

    //上升
    case Phase::ASCEND: {

        auto current = mav_.getCurrentPose();
        double current_z = current.pose.position.z;

        //到达位置后停止
        if (current_z >= target_z_) {
            ROS_INFO("TakeoffNode: Reached target altitude (%.2f >= %.2f)!",
                     current_z, target_z_);
            phase_ = Phase::DONE;
            return BT::NodeStatus::RUNNING;
        }

        return BT::NodeStatus::RUNNING;
    }

    case Phase::DONE:
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void TakeoffNode::onHalted() {
    ROS_WARN("TakeoffNode: Halted! Holding position.");
    mav_.setSetpointMode(SetpointMode::HEARTBEAT);
}

} // namespace maxt
