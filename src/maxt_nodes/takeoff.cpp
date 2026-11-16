#include <maxt_pkg/maxt_nodes/takeoff_node.hpp>
#include <cmath>

namespace maxt {

BT::PortsList TakeoffNode::providedPorts() {
    return {
        BT::InputPort<double>("target_alt", 0.5, "takeoff altitude"),
        BT::OutputPort<double>("home_x", "takeoff X in calibrated navigation frame"),
        BT::OutputPort<double>("home_y", "takeoff Y in calibrated navigation frame"),
        BT::OutputPort<double>("home_z", "takeoff Z in calibrated navigation frame"),
        BT::OutputPort<double>("return_z", "takeoff Z plus target_alt in calibrated navigation frame")
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
    params.param("settle_time", settle_time_, 0.4);
    ros::NodeHandle reach("~reach_check");
    reach.param("vertical_alt_tolerance", altitude_tolerance_, 0.1);
    reach.param("vertical_vel_tolerance", vertical_velocity_tolerance_, 0.2);

    if (!getInput<double>("target_alt", target_alt_)) {
        ROS_ERROR("TakeoffNode: Invalid target_alt input.");
        return BT::NodeStatus::FAILURE;
    }
    for (double value : {target_alt_, ascent_speed_, timeout_, settle_time_,
                         altitude_tolerance_, vertical_velocity_tolerance_}) {
        if (!std::isfinite(value) || value <= 0.0) {
            ROS_ERROR("TakeoffNode: height, speed, timeout and settling limits must be finite and positive.");
            return BT::NodeStatus::FAILURE;
        }
    }

    hold_yaw_ = mav_.get_current_yaw();
    mav_.setDefaultYaw(hold_yaw_);

    phase_ = Phase::WAIT_ARM;
    start_time_ = ros::SteadyTime::now();
    stable_since_ = ros::SteadyTime();
    target_initialized_ = false;

    mav_.setSetpointMode(SetpointMode::HEARTBEAT);
    ROS_INFO("TakeoffNode: Waiting for external arming, target_alt=%.2f m, ascent_speed=%.2f m/s",
             target_alt_, ascent_speed_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakeoffNode::onRunning() {

    const auto now = ros::SteadyTime::now();
    if (target_initialized_) {
        const auto pose = mav_.GetRawPose();
        const auto& p = pose.pose.position;
        if (!pose.header.stamp.isZero() && std::isfinite(p.x) &&
            std::isfinite(p.y) && std::isfinite(p.z)) {
            last_valid_position_ = p;
        }
    }

    // 持续等待外部解锁；解锁后才开始 OFFBOARD 和爬升超时计时。
    if (phase_ != Phase::WAIT_ARM && (now - start_time_).toSec() > timeout_) {
        ROS_ERROR("TakeoffNode: Timeout (%.1f s)!", timeout_);
        stopClimb();
        return BT::NodeStatus::FAILURE;
    }

    // Once climbing, loss of feedback or pilot takeover must remove the positive
    // vertical-velocity command. Never request OFFBOARD again from these phases.
    if ((phase_ == Phase::ASCEND || phase_ == Phase::SETTLE) &&
        (!mav_.hasFreshNavigationData() || !mav_.isArmed() || mav_.getMode() != "OFFBOARD")) {
        ROS_ERROR("TakeoffNode: feedback unavailable, disarmed, or OFFBOARD exited.");
        stopClimb();
        return BT::NodeStatus::FAILURE;
    }

    switch (phase_) {

    // 只等待外部解锁，不主动请求 arming。
    case Phase::WAIT_ARM: {
        if (!mav_.isArmed()) {
            return BT::NodeStatus::RUNNING;
        }
        start_time_ = now;
        ROS_INFO("TakeoffNode: Armed, switching to OFFBOARD...");
        // 确认已解锁后只触发一次，随阶段切换避免每个 tick 重复播报。
        mav_.playAudio("cn-armed.wav");
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
        mav_.playAudio("cn-takeoff.wav");
        ROS_INFO("TakeoffNode: OFFBOARD confirmed");
        phase_ = Phase::INIT;
        return BT::NodeStatus::RUNNING;
    }

    //初始化
    case Phase::INIT: {
        const auto home = mav_.GetCalibratePose();
        // 尚未收到定位时等待，避免将默认零坐标写入黑板；受起飞超时限制。
        if (home.header.stamp.isZero() || !mav_.hasFreshNavigationData()) {
            return BT::NodeStatus::RUNNING;
        }
        // 输出可选，兼容未配置黑板映射的已有任务树。仅在爬升前记录一次。
        const auto write_output = [this](const std::string& port, double value) {
            if (config().output_ports.count(port)) {
                setOutput(port, value);
            }
        };
        write_output("home_x", home.pose.position.x);
        write_output("home_y", home.pose.position.y);
        write_output("home_z", home.pose.position.z);
        write_output("return_z", home.pose.position.z + target_alt_);
        mav_.RecordRawHomePose();
        const auto current = mav_.GetRawPose();
        last_valid_position_ = current.pose.position;
        aim_altitude_raw_ = current.pose.position.z + target_alt_;
        if (!std::isfinite(aim_altitude_raw_)) return BT::NodeStatus::FAILURE;
        goal_raw_ = current;
        goal_raw_.pose.position.z = aim_altitude_raw_;
        hold_target_ = mavros_msgs::PositionTarget();
        hold_target_.header.frame_id = "map";
        hold_target_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        hold_target_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        hold_target_.position = goal_raw_.pose.position;
        hold_target_.yaw = hold_yaw_;
        target_initialized_ = true;

        mavros_msgs::PositionTarget takeoff_raw_target;
        takeoff_raw_target.header.frame_id = "map";
        takeoff_raw_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        takeoff_raw_target.type_mask =
            mavros_msgs::PositionTarget::IGNORE_PZ |
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        takeoff_raw_target.position.x = current.pose.position.x;
        takeoff_raw_target.position.y = current.pose.position.y;
        takeoff_raw_target.yaw = hold_yaw_;
        takeoff_raw_target.velocity.z = ascent_speed_;
        mav_.setRawTarget(takeoff_raw_target);
        mav_.setSetpointMode(SetpointMode::RAW_CTRL);

        ROS_INFO("TakeoffNode: Ascending from z=%.2f to z=%.2f at %.2f m/s, yaw=%.2f",
                 current.pose.position.z, aim_altitude_raw_, ascent_speed_, hold_yaw_);
        mav_.playAudio("sample-6s.wav");
        phase_ = Phase::ASCEND;
        return BT::NodeStatus::RUNNING;
    }

    //上升
    case Phase::ASCEND: {

        // A sampled position may jump from below to ABOVE the tolerance band.
        // A one-sided threshold catches both cases; immediately disable VZ and
        // enable the fixed target Z, instead of waiting another tick to brake.
        if (mav_.GetRawPose().pose.position.z >= aim_altitude_raw_ - altitude_tolerance_) {
            holdTarget();
            stable_since_ = ros::SteadyTime();
            phase_ = Phase::SETTLE;
            ROS_INFO("TakeoffNode: Switching from ascent to target-height hold at z=%.2f",
                     aim_altitude_raw_);
        }
        return BT::NodeStatus::RUNNING;
    }

    case Phase::SETTLE: {
        holdTarget();
        const auto velocity = mav_.getCurrentTwist().twist.linear;
        if (!mav_.ReachAltitudeCheck(aim_altitude_raw_) || !mav_.ReachPoseCheck(goal_raw_) ||
            std::abs(velocity.z) > vertical_velocity_tolerance_) {
            stable_since_ = ros::SteadyTime();
            return BT::NodeStatus::RUNNING;
        }
        if (stable_since_.isZero()) stable_since_ = now;
        if ((now-stable_since_).toSec() < settle_time_) return BT::NodeStatus::RUNNING;
        ROS_INFO("TakeoffNode: Target height and low velocity stable for %.2f s.", settle_time_);
        // Keep the fixed position setpoint active until the next BT action takes over.
        return BT::NodeStatus::SUCCESS;
    }
    }

    return BT::NodeStatus::RUNNING;
}

void TakeoffNode::onHalted() {
    ROS_WARN("TakeoffNode: Halted! Holding position.");
    stopClimb();
}

void TakeoffNode::holdTarget() {
    mav_.setRawTarget(hold_target_, false);
    mav_.setSetpointMode(SetpointMode::RAW_CTRL);
}

void TakeoffNode::stopClimb() {
    if (!target_initialized_) {
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return;
    }
    // Abort at the last measured position, not at an unreached ascent goal.
    // The mask also disables velocity, so no old positive VZ remains active.
    hold_target_.position = last_valid_position_;
    holdTarget();
}

} // namespace maxt
