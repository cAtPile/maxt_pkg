#include <maxt_pkg/maxt_nodes/la_lo_al_nav_node.hpp>

namespace maxt {

BT::PortsList LaLoAlNavNode::providedPorts() {
    return {
        BT::InputPort<double>("latitude"),
        BT::InputPort<double>("longitude"),
        BT::InputPort<double>("altitude"),
        BT::InputPort<double>("timeout", 60.0, "timeout (s)")
    };
}

BT::NodeStatus LaLoAlNavNode::onStart() {
    nh_.param<double>("nav/hold_timeout", hold_timeout_, 3.0);
    double altitude;
    if (!getInput<double>("latitude", target_.latitude) ||
        !getInput<double>("longitude", target_.longitude) ||
        !getInput<double>("altitude", altitude)) {
        ROS_ERROR("[LaLoAlNav] Missing latitude/longitude/altitude.");
        return BT::NodeStatus::FAILURE;
    }
    if (!mav_.hasGlobalPose()) {
        ROS_ERROR("[LaLoAlNav] Waiting for valid global position, relative altitude and local pose.");
        return BT::NodeStatus::FAILURE;
    }
    getInput<double>("timeout", timeout_);
    // XML 高度相对起飞点；到达检查和 PID 使用原始 local Z。
    altitude_raw_ = mav_.getHomePose().pose.position.z + altitude;
    const double current_local_z = mav_.GetRawPose().pose.position.z;
    const auto start = mav_.GetGlobalPose();
    // 飞控相对 Home 高度和 local Z 的零点可能不同，不能直接互换。
    target_.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    target_.altitude = start.altitude + altitude_raw_ - current_local_z;
    double kp, ki, kd, v_max, a_max;
    if (!nh_.getParam("nav/pid_kp", kp) || !nh_.getParam("nav/pid_ki", ki) ||
        !nh_.getParam("nav/pid_kd", kd)) {
        ROS_ERROR("[LaLoAlNav] Missing PID configuration; load config/nav.yaml.");
        return BT::NodeStatus::FAILURE;
    }
    nh_.param<double>("nav/v_max", v_max, 2.5);
    nh_.param<double>("nav/a_max", a_max, 1.2);
    curve_.setTarget(start, target_, v_max, a_max);
    for (auto* pid : {&pid_east_, &pid_north_, &pid_altitude_}) {
        pid->reset();
        pid->setGains(kp, ki, kd);
        pid->setOutputClamp(0.5);
        pid->setIntegralClamp(0.3);
    }
    start_time_ = ros::Time::now();
    last_time_ = start_time_;
    holding_ = false;
    mav_.SetGlobalRawTarget(curve_.sample(0.0));
    mav_.setSetpointMode(SetpointMode::GLOBAL_RAW_CTRL);
    ROS_INFO("[LaLoAlNav] START: lat=%.8f lon=%.8f height=%.2f local_z=%.2f T=%.2fs",
             target_.latitude, target_.longitude, altitude, altitude_raw_, curve_.duration());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LaLoAlNavNode::onRunning() {
    const auto now = ros::Time::now();
    const double elapsed = (now - start_time_).toSec();
    if (elapsed > timeout_) {
        ROS_ERROR("[LaLoAlNav] Timeout.");
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::FAILURE;
    }
    const double dt = (now - last_time_).toSec();
    if (dt <= 1e-6) return BT::NodeStatus::RUNNING;
    last_time_ = now;
    if (!mav_.hasGlobalPose()) {
        ROS_ERROR("[LaLoAlNav] Global position, relative altitude or local pose unavailable.");
        mav_.setSetpointMode(SetpointMode::SLOW_DOWN);
        return BT::NodeStatus::FAILURE;
    }
    if (elapsed < curve_.duration()) {
        mav_.SetGlobalRawTarget(curve_.sample(elapsed));
        return BT::NodeStatus::RUNNING;
    }
    // 与 Nav 一致，从首次进入末端对齐阶段开始计时。
    if (!holding_) {
        holding_ = true;
        hold_start_ = now;
    }
    if (mav_.ReachGlobalCheck(target_, altitude_raw_)) {
        ROS_INFO("[LaLoAlNav] REACHED.");
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }
    const double hold_elapsed = (now - hold_start_).toSec();
    if (hold_elapsed > hold_timeout_) {
        ROS_WARN("[LaLoAlNav] Hold timeout (%.1fs). Forcing SUCCESS.", hold_elapsed);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }
    // 保持最终经纬度和相对高度设点；高度 PID 使用 local Z 误差。
    const auto pose = mav_.GetGlobalPose();
    double distance, azimuth, end_azimuth;
    GeographicLib::Geodesic::WGS84().Inverse(
        pose.latitude, pose.longitude, target_.latitude, target_.longitude,
        distance, azimuth, end_azimuth);
    const double heading = azimuth * M_PI / 180.0;
    const double error_east = distance * std::sin(heading);
    const double error_north = distance * std::cos(heading);
    auto target = curve_.sample(curve_.duration());
    // 固定零设点、负误差测量值，使现有 PID 的测量微分也能正确作用。
    target.velocity.x = pid_east_.update(0.0, -error_east, dt);
    target.velocity.y = pid_north_.update(0.0, -error_north, dt);
    target.velocity.z = pid_altitude_.update(altitude_raw_, mav_.GetRawPose().pose.position.z, dt);
    mav_.SetGlobalRawTarget(target);
    return BT::NodeStatus::RUNNING;
}

void LaLoAlNavNode::onHalted() {
    ROS_WARN("[LaLoAlNav] Halted! Slowing down.");
    mav_.setSetpointMode(SetpointMode::SLOW_DOWN);
}

} // namespace maxt
