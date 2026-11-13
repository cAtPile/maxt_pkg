#include <maxt_pkg/maxt_nodes/nav/nav_geo_node.hpp>
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace maxt {
namespace {
double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
double yawOf(const geometry_msgs::PoseStamped& pose) {
    const auto& q = pose.pose.orientation;
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
    return yaw;
}
} // namespace

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
    double azimuth_angle;
    nh_.param<double>("nav/azimuth_angle", azimuth_angle, 0.0);
    if (!std::isfinite(azimuth_angle)) {
        ROS_ERROR("[NavGeo] nav/azimuth_angle must be finite.");
        return BT::NodeStatus::FAILURE;
    }
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
    const auto raw_pose = mav_.GetRawPose();
    altitude_raw_ = raw_pose.pose.position.z;
    const auto& q = raw_pose.pose.orientation;
    const double norm2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(norm2) || norm2 < 1e-12) {
        ROS_ERROR("[NavGeo] Invalid raw orientation.");
        return BT::NodeStatus::FAILURE;
    }
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
    double yaw_rate, yaw_acc, tolerance_deg;
    nh_.param<double>("re_heading/yaw_rate_max", yaw_rate, 0.5);
    nh_.param<double>("re_heading/yaw_acc_max", yaw_acc, 0.5);
    nh_.param<double>("re_heading/tolerance_deg", tolerance_deg, 2.0);
    if (!std::isfinite(yaw_rate) || yaw_rate <= 0.0 ||
        !std::isfinite(yaw_acc) || yaw_acc <= 0.0 ||
        !std::isfinite(tolerance_deg) || tolerance_deg <= 0.0 ||
        !std::isfinite(v_max) || v_max <= 0.0 ||
        !std::isfinite(a_max) || a_max <= 0.0 ||
        !std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd) ||
        !std::isfinite(timeout_) || timeout_ <= 0.0 ||
        !std::isfinite(hold_timeout_) || hold_timeout_ < 0.0) {
        ROS_ERROR("[NavGeo] Invalid yaw/PID/timeout configuration.");
        return BT::NodeStatus::FAILURE;
    }
    last_yaw_ = measured_yaw_ = yawOf(raw_pose);
    target_yaw_ = wrap(std::remainder(azimuth_angle, 360.0) * M_PI / 180.0);
    const double delta = wrap(target_yaw_ - measured_yaw_);
    yaw_tolerance_ = tolerance_deg * M_PI / 180.0;
    yaw_duration_ = 1.2 * std::max({1.875 * std::abs(delta) / (0.9 * yaw_rate),
                                  std::sqrt(5.774 * std::abs(delta) / yaw_acc), 0.5});
    if (!std::isfinite(yaw_duration_)) {
        ROS_ERROR("[NavGeo] Invalid yaw curve duration.");
        return BT::NodeStatus::FAILURE;
    }
    QuinticCurve::solveAxis(measured_yaw_, 0.0, 0.0, measured_yaw_ + delta, 0.0, 0.0,
                            yaw_duration_, yaw_coefficients_);
    pid_yaw_.reset();
    pid_yaw_.setGains(kp, ki, kd);
    pid_yaw_.setOutputClamp(yaw_rate);
    pid_yaw_.setIntegralClamp(0.3);
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
    auto initial_target = curve_.sample(0.0);
    applyYaw(initial_target, 0.0, 0.0);
    mav_.SetGlobalRawTarget(initial_target, false);
    mav_.setSetpointMode(SetpointMode::GLOBAL_RAW_CTRL);
    ROS_INFO("[NavGeo] START: lat=%.8f lon=%.8f hold_local_z=%.2f T=%.2fs yaw_T=%.2fs",
             target_.latitude, target_.longitude, altitude_raw_, curve_.duration(), yaw_duration_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavGeo::onRunning() {
    const auto now = ros::Time::now();
    const double elapsed = (now - start_time_).toSec();
    if (elapsed > timeout_) {
        ROS_ERROR("[NavGeo] Timeout.");
        mav_.setDefaultYaw(yawOf(mav_.GetRawPose()));
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::FAILURE;
    }
    const double dt = (now - last_time_).toSec();
    if (dt <= 1e-6) return BT::NodeStatus::RUNNING;
    last_time_ = now;
    if (!mav_.hasGlobalPose()) {
        ROS_ERROR("[NavGeo] Global position, relative altitude or local pose unavailable.");
        mav_.setDefaultYaw(yawOf(mav_.GetRawPose()));
        mav_.setSetpointMode(SetpointMode::SLOW_DOWN);
        return BT::NodeStatus::FAILURE;
    }
    const double yaw = yawOf(mav_.GetRawPose());
    measured_yaw_ += wrap(yaw - last_yaw_);
    last_yaw_ = yaw;
    if (elapsed < curve_.duration()) {
        auto target = curve_.sample(elapsed);
        applyYaw(target, elapsed, dt);
        mav_.SetGlobalRawTarget(target, false);
        return BT::NodeStatus::RUNNING;
    }
    // 平移和航向曲线均结束后，才开始计算末端对齐超时。
    const bool curves_finished = elapsed >= yaw_duration_;
    if (curves_finished && !holding_) {
        holding_ = true;
        hold_start_ = now;
    }
    const double yaw_error = wrap(target_yaw_ - yaw);
    if (curves_finished && mav_.ReachGlobalCheck(target_, altitude_raw_) &&
        std::abs(yaw_error) < yaw_tolerance_) {
        ROS_INFO("[NavGeo] REACHED.");
        mav_.setDefaultYaw(target_yaw_);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }
    const double hold_elapsed = holding_ ? (now - hold_start_).toSec() : 0.0;
    if (holding_ && hold_elapsed > hold_timeout_) {
        ROS_WARN("[NavGeo] Hold timeout (%.1fs), yaw_err=%.2f deg. Forcing SUCCESS.",
                 hold_elapsed, yaw_error * 180.0 / M_PI);
        mav_.setDefaultYaw(target_yaw_);
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
    applyYaw(target, elapsed, dt);
    mav_.SetGlobalRawTarget(target, false);
    return BT::NodeStatus::RUNNING;
}



void NavGeo::applyYaw(mavros_msgs::GlobalPositionTarget& target, double elapsed, double dt) {
    using Target = mavros_msgs::GlobalPositionTarget;
    target.type_mask &= ~(Target::IGNORE_YAW | Target::IGNORE_YAW_RATE);
    if (elapsed < yaw_duration_) {
        target.yaw = wrap(QuinticCurve::evalPos(yaw_coefficients_, elapsed));
        target.yaw_rate = QuinticCurve::evalVel(yaw_coefficients_, elapsed);
    } else {
        // 展开测量角以避免跨越 +/-pi 时产生微分跳变；按最近等价角修正。
        const double error = wrap(target_yaw_ - measured_yaw_);
        target.type_mask |= Target::IGNORE_YAW;
        target.yaw = target_yaw_;
        target.yaw_rate = pid_yaw_.update(measured_yaw_ + error, measured_yaw_, dt);
    }
}

void NavGeo::onHalted() {
    mav_.setDefaultYaw(yawOf(mav_.GetRawPose()));
    recordBreakpointAndSlowDown();
}

} // namespace maxt
