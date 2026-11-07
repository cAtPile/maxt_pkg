#include <maxt_pkg/maxt_nodes/la_lo_nav_node.hpp>

namespace maxt {

BT::PortsList LaLoNavNode::providedPorts() {
    return {
        BT::InputPort<double>("latitude"),
        BT::InputPort<double>("longitude"),
        BT::InputPort<double>("timeout", 60.0, "timeout (s)")
    };
}

BT::NodeStatus LaLoNavNode::onStart() {
    nh_.param<double>("nav/hold_timeout", hold_timeout_, 3.0);
    nh_.param<double>("nav/lalo_safety_xy_limit", safety_xy_limit_, 10.0);
    if (!std::isfinite(safety_xy_limit_) || safety_xy_limit_ <= 0.0) {
        ROS_ERROR("[LaLoNav] nav/lalo_safety_xy_limit must be finite and positive.");
        return BT::NodeStatus::FAILURE;
    }
    if (!getInput<double>("latitude", target_.latitude) ||
        !getInput<double>("longitude", target_.longitude)) {
        ROS_ERROR("[LaLoNav] Missing latitude/longitude.");
        return BT::NodeStatus::FAILURE;
    }
    if (!mav_.hasGlobalPose()) {
        ROS_ERROR("[LaLoNav] Waiting for valid global position, relative altitude and local pose.");
        return BT::NodeStatus::FAILURE;
    }
    getInput<double>("timeout", timeout_);
    // 固定启动时的相对高度设点；到达检查使用同一时刻附近的 local Z。
    start_local_pose_ = mav_.GetRawPose();
    altitude_raw_ = start_local_pose_.pose.position.z;
    const auto start = mav_.GetGlobalPose();
    target_.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    target_.altitude = start.altitude;
    double kp, ki, kd, v_max, a_max;
    if (!nh_.getParam("nav/pid_kp", kp) || !nh_.getParam("nav/pid_ki", ki) ||
        !nh_.getParam("nav/pid_kd", kd)) {
        ROS_ERROR("[LaLoNav] Missing PID configuration; load config/nav.yaml.");
        return BT::NodeStatus::FAILURE;
    }
    nh_.param<double>("nav/v_max", v_max, 2.5);
    nh_.param<double>("nav/a_max", a_max, 1.2);
    curve_.setTarget(start, target_, v_max, a_max);
    for (auto* pid : {&pid_east_, &pid_north_}) {
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
    ROS_INFO("[LaLoNav] START: lat=%.8f lon=%.8f hold_local_z=%.2f T=%.2fs",
             target_.latitude, target_.longitude, altitude_raw_, curve_.duration());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LaLoNavNode::onRunning() {
    // 安全门优先于超时和强制成功；使用原始 local 坐标，不受 Home 偏移影响。
    const auto local_pose = mav_.GetRawPose();
    const double dx = local_pose.pose.position.x - start_local_pose_.pose.position.x;
    const double dy = local_pose.pose.position.y - start_local_pose_.pose.position.y;
    if (std::abs(dx) > safety_xy_limit_ || std::abs(dy) > safety_xy_limit_) {
        ROS_ERROR("[LaLoNav] XY safety limit exceeded: dx=%.2f dy=%.2f limit=+/-%.2f m. Slowing down.",
                  dx, dy, safety_xy_limit_);
        mav_.setSetpointMode(SetpointMode::SLOW_DOWN);
        return BT::NodeStatus::FAILURE;
    }
    const auto now = ros::Time::now();
    const double elapsed = (now - start_time_).toSec();
    if (elapsed > timeout_) {
        ROS_ERROR("[LaLoNav] Timeout.");
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::FAILURE;
    }
    const double dt = (now - last_time_).toSec();
    if (dt <= 1e-6) return BT::NodeStatus::RUNNING;
    last_time_ = now;
    if (!mav_.hasGlobalPose()) {
        ROS_ERROR("[LaLoNav] Global position, relative altitude or local pose unavailable.");
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
        ROS_INFO("[LaLoNav] REACHED.");
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }
    const double hold_elapsed = (now - hold_start_).toSec();
    if (hold_elapsed > hold_timeout_) {
        ROS_WARN("[LaLoNav] Hold timeout (%.1fs). Forcing SUCCESS.", hold_elapsed);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }
    // 保持启动高度，仅对东、北方向做 PID 末端对齐。
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
    mav_.SetGlobalRawTarget(target);
    return BT::NodeStatus::RUNNING;
}

void LaLoNavNode::onHalted() {
    ROS_WARN("[LaLoNav] Halted! Slowing down.");
    mav_.setSetpointMode(SetpointMode::SLOW_DOWN);
}

} // namespace maxt
