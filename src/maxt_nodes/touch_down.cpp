#include <maxt_pkg/maxt_nodes/touch_down_node.hpp>
#include <cmath>

namespace maxt {

BT::PortsList TouchDownNode::providedPorts() {
    return {
        BT::InputPort<double>("descent_speed", -0.5, "下降速度(m/s)"),
        BT::InputPort<double>("timeout", 60.0, "降落总超时(s)"),
        BT::InputPort<double>("ground_timeout", 3.0, "着陆确认持续时间(s)"),
        BT::InputPort<double>("height_tol", 0.3, "高度容差(m), 相对home点"),
        BT::InputPort<double>("vel_tol", 0.2, "垂速容差(m/s)")
    };
}

TouchDownNode::TouchDownNode(const std::string& name,
                             const BT::NodeConfiguration& config,
                             MavKit& mav)
    : MavActionNode(name, config, mav) {
}

BT::NodeStatus TouchDownNode::onStart() {
    if (!getInput<double>("descent_speed", descent_speed_)) descent_speed_ = -0.5;
    if (!getInput<double>("timeout", timeout_)) timeout_ = 60.0;
    if (!getInput<double>("ground_timeout", ground_timeout_)) ground_timeout_ = 3.0;
    if (!getInput<double>("height_tol", height_tol_)) height_tol_ = 0.3;
    if (!getInput<double>("vel_tol", vel_tol_)) vel_tol_ = 0.2;

    if (!mav_.isArmed()) {
        ROS_INFO("TouchDownNode: Already disarmed");
        return BT::NodeStatus::SUCCESS;
    }

    phase_ = Phase::INIT;
    start_time_ = ros::Time::now();
    land_confirming_ = false;
    home_z_ = mav_.getHomePose().pose.position.z;

    ROS_INFO("TouchDownNode: Start, home_z=%.2f, descent_speed=%.2f, timeout=%.1f, "
             "ground_timeout=%.1f, height_tol=%.2f, vel_tol=%.2f",
             home_z_, descent_speed_, timeout_, ground_timeout_, height_tol_, vel_tol_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TouchDownNode::onRunning() {
    ros::Time now = ros::Time::now();

    // 超时强制 disarm，不切模式
    if ((now - start_time_).toSec() > timeout_) {
        ROS_ERROR("TouchDownNode: Timeout (%.1f s)! Forced disarm.", timeout_);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        mav_.requestArm(false);
        disarm_request_time_ = now;
        phase_ = Phase::DISARM;
        return BT::NodeStatus::RUNNING;
    }

    switch (phase_) {

    case Phase::INIT: {
        auto current = mav_.getCurrentPose();
        hold_x_ = current.pose.position.x;
        hold_y_ = current.pose.position.y;

        mavros_msgs::PositionTarget target;
        target.header.frame_id = "map";
        target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target.type_mask =
            mavros_msgs::PositionTarget::IGNORE_PZ |
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW;
        target.position.x = hold_x_;
        target.position.y = hold_y_;
        target.velocity.z = descent_speed_;
        target.yaw_rate = 0.0;
        mav_.setRawTarget(target);

        mav_.setSetpointMode(SetpointMode::RAW_CTRL);

        ROS_INFO("TouchDownNode: Descending from (%.2f, %.2f) at %.2f m/s",
                 hold_x_, hold_y_, descent_speed_);
        phase_ = Phase::DESCEND;
        return BT::NodeStatus::RUNNING;
    }

    case Phase::DESCEND: {
        auto current = mav_.getCurrentPose();
        auto twist = mav_.getCurrentTwist();
        double current_z = current.pose.position.z;
        double vz = twist.twist.linear.z;

        // 联合判断：高度、垂速、extended_state
        bool height_ok = std::fabs(current_z - home_z_) < height_tol_;
        bool vel_ok = std::fabs(vz) < vel_tol_;
        bool land_ok = mav_.isLand();

        if (height_ok && vel_ok && land_ok) {
            if (!land_confirming_) {
                land_confirming_ = true;
                land_confirm_time_ = now;
                ROS_INFO("TouchDownNode: Landing candidate: z=%.2f(home=%.2f) vz=%.2f isLand=%d",
                         current_z, home_z_, vz, land_ok);
            }

            double elapsed = (now - land_confirm_time_).toSec();
            if (elapsed > ground_timeout_) {
                ROS_INFO("TouchDownNode: Landing confirmed! Disarm.");
                mav_.setSetpointMode(SetpointMode::HEARTBEAT);
                mav_.requestArm(false);
                disarm_request_time_ = now;
                phase_ = Phase::DISARM;
                return BT::NodeStatus::RUNNING;
            }
        } else {
            land_confirming_ = false;
        }

        return BT::NodeStatus::RUNNING;
    }

    case Phase::DISARM: {
        if (!mav_.isArmed()) {
            ROS_INFO("TouchDownNode: Disarmed, touchdown complete.");
            phase_ = Phase::DONE;
            return BT::NodeStatus::SUCCESS;
        }

        if ((now - disarm_request_time_).toSec() > 2.0) {
            ROS_WARN("TouchDownNode: Retrying disarm...");
            mav_.requestArm(false);
            disarm_request_time_ = now;
        }

        return BT::NodeStatus::RUNNING;
    }

    case Phase::DONE:
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void TouchDownNode::onHalted() {
    ROS_WARN("TouchDownNode: Halted! Switching to HEARTBEAT.");
    mav_.setSetpointMode(SetpointMode::HEARTBEAT);
}

}  // namespace maxt
