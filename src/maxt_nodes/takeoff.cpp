//TODO 需要构造 MavKit::GetHomeRawAltitude 获取起飞点的高度
#include <maxt_pkg/maxt_nodes/takeoff_node.hpp>

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

    if (!getInput<double>("target_alt", target_alt_)) target_alt_ = 0.5;

    hold_yaw_ = mav_.get_current_yaw();
    mav_.setDefaultYaw(hold_yaw_);

    phase_ = Phase::WAIT_ARM;
    start_time_ = ros::Time::now();

    mav_.setSetpointMode(SetpointMode::HEARTBEAT);
    ROS_INFO("TakeoffNode: Waiting for external arming, target_alt=%.2f m, ascent_speed=%.2f m/s",
             target_alt_, ascent_speed_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakeoffNode::onRunning() {

    ros::Time now = ros::Time::now();

    // 持续等待外部解锁；解锁后才开始 OFFBOARD 和爬升超时计时。
    if (phase_ != Phase::WAIT_ARM && (now - start_time_).toSec() > timeout_) {
        ROS_ERROR("TakeoffNode: Timeout (%.1f s)!", timeout_);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
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
        if (home.header.stamp.isZero()) {
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
        aim_altitude_raw_ = current.pose.position.z + target_alt_;

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

        // 使用原始局部坐标高度判定，不应用 Home 偏移或标定。
        if (mav_.ReachAltitudeCheck(aim_altitude_raw_)) {
            ROS_INFO("TakeoffNode: Reached target raw altitude %.2f m!", aim_altitude_raw_);
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
