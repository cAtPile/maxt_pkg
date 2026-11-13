#include <maxt_pkg/maxt_nodes/nav/nav_geo_node.hpp>
#include <cmath>

namespace maxt {

BT::PortsList NavGeo::providedPorts() {
    return {
        BT::OutputPort<double>("breakpoint_x"),
        BT::OutputPort<double>("breakpoint_y"),
        BT::OutputPort<double>("breakpoint_z"),
        BT::OutputPort<bool>("breakpoint_valid"),
        BT::InputPort<double>("latitude"),
        BT::InputPort<double>("longitude"),
        BT::InputPort<double>("timeout", 60.0, "timeout (s)")
    };
}

BT::NodeStatus NavGeo::onStart() {
    nh_.param<double>("nav/hold_timeout", hold_timeout_, 3.0);
    nh_.param<double>("nav/lalo_max_duration", max_duration_, 60.0);
    if (!std::isfinite(max_duration_) || max_duration_ <= 0.0) {
        ROS_ERROR("[NavGeo] nav/lalo_max_duration must be finite and positive.");
        return BT::NodeStatus::FAILURE;
    }
    if (!getInput<double>("latitude", target_.latitude) ||
        !getInput<double>("longitude", target_.longitude)) {
        ROS_ERROR("[NavGeo] Missing latitude/longitude.");
        return BT::NodeStatus::FAILURE;
    }
    if (!mav_.hasGlobalPose()) {
        ROS_ERROR("[NavGeo] Waiting for valid global position, relative altitude and local pose.");
        return BT::NodeStatus::FAILURE;
    }
    getInput<double>("timeout", timeout_);
    // 固定启动时的相对高度设点；到达检查使用同一时刻附近的 local Z。
    altitude_raw_ = mav_.GetRawPose().pose.position.z;
    const auto start = mav_.GetGlobalPose();
    target_.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    target_.altitude = start.altitude;
    double kp, ki, kd, v_max, a_max;
    if (!nh_.getParam("nav/pid_kp", kp) || !nh_.getParam("nav/pid_ki", ki) ||
        !nh_.getParam("nav/pid_kd", kd)) {
        ROS_ERROR("[NavGeo] Missing PID configuration; load config/nav.yaml.");
        return BT::NodeStatus::FAILURE;
    }
    nh_.param<double>("nav/v_max", v_max, 2.5);
    nh_.param<double>("nav/a_max", a_max, 1.2);
    curve_.setTarget(start, target_, v_max, a_max);
    // 在发送设点前检查预计曲线时长，拦截错误经纬度造成的长距离航行。
    if (curve_.duration() > max_duration_) {
        ROS_ERROR("[NavGeo] Estimated duration T=%.2f s exceeds limit %.2f s.",
                  curve_.duration(), max_duration_);
        return BT::NodeStatus::FAILURE;
    }
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
    ROS_INFO("[NavGeo] START: lat=%.8f lon=%.8f hold_local_z=%.2f T=%.2fs",
             target_.latitude, target_.longitude, altitude_raw_, curve_.duration());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavGeo::onRunning() {
    const auto now = ros::Time::now();
    const double elapsed = (now - start_time_).toSec();
    if (elapsed > timeout_) {
        ROS_ERROR("[NavGeo] Timeout.");
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::FAILURE;
    }
    const double dt = (now - last_time_).toSec();
    if (dt <= 1e-6) return BT::NodeStatus::RUNNING;
    last_time_ = now;
    if (!mav_.hasGlobalPose()) {
        ROS_ERROR("[NavGeo] Global position, relative altitude or local pose unavailable.");
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
        ROS_INFO("[NavGeo] REACHED.");
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }
    const double hold_elapsed = (now - hold_start_).toSec();
    if (hold_elapsed > hold_timeout_) {
        ROS_WARN("[NavGeo] Hold timeout (%.1fs). Forcing SUCCESS.", hold_elapsed);
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



void NavGeo::onHalted() {
    recordBreakpointAndSlowDown();
}

} // namespace maxt
