#include <maxt_pkg/maxt_nodes/takeoff_node.hpp>
#include <cmath>

namespace maxt {

BT::PortsList TakeoffNode::providedPorts() {
    return {
        BT::InputPort<double>("target_alt", 0.5, "takeoff altitude")
    };
}

TakeoffNode::TakeoffNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav) {
}

BT::NodeStatus TakeoffNode::onStart() {
    // 每次启动从节点私有命名空间读取本模块的 ROS 参数。
    ros::NodeHandle params("~takeoff");
    params.param("ascent_speed", ascent_speed_, 0.5);
    params.param("timeout", timeout_, 30.0);
    params.param("arm_timeout", arm_timeout_, 60.0);
    if (!std::isfinite(arm_timeout_) || arm_timeout_ <= 0.0) {
        ROS_ERROR("TakeoffNode: arm_timeout must be finite and positive!");
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput<double>("target_alt", target_alt_)) target_alt_ = 0.5;

    hold_yaw_ = mav_.get_current_yaw();
    mav_.setDefaultYaw(hold_yaw_);

    phase_ = Phase::WAIT_ARM;
    start_time_ = ros::Time::now();

    mav_.setSetpointMode(SetpointMode::HEARTBEAT);
    ROS_INFO("TakeoffNode: Waiting for external arming (timeout=%.1f s), target_alt=%.2f m, ascent_speed=%.2f m/s",
             arm_timeout_, target_alt_, ascent_speed_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakeoffNode::onRunning() {

    ros::Time now = ros::Time::now();

    // 解锁等待独立计时；解锁后才开始 OFFBOARD 和爬升超时计时。
    if (phase_ != Phase::WAIT_ARM && (now - start_time_).toSec() > timeout_) {
        ROS_ERROR("TakeoffNode: Timeout (%.1f s)!", timeout_);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::FAILURE;
    }

    switch (phase_) {

    // 只等待外部解锁，不主动请求 arming。
    case Phase::WAIT_ARM: {
        if (!mav_.isArmed()) {
            if ((now - start_time_).toSec() > arm_timeout_) {
                ROS_ERROR("TakeoffNode: Waiting for arming timed out (%.1f s)!", arm_timeout_);
                mav_.setSetpointMode(SetpointMode::HEARTBEAT);
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }
        start_time_ = now;
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
