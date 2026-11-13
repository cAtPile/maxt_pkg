#include <maxt_pkg/mavkit/maxt_mav.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <cmath>

namespace maxt {

void MavKit::setDefaultYaw(double yaw) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    default_yaw_ = yaw;
    has_default_yaw_ = true;
    ROS_INFO("MavKit: Default yaw locked to %.4f rad (%.2f deg)",
             yaw, yaw * 180.0 / M_PI);
}

//todo 相对化处理home
void MavKit::setTargetPose(double x, double y, double z) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    target_pose_.header.stamp = ros::Time::now();
    target_pose_.header.frame_id = "map"; // 或者是你的坐标系名称
    target_pose_.pose.position.x = x;
    target_pose_.pose.position.y = y;
    target_pose_.pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, default_yaw_);
    target_pose_.pose.orientation.x = q.x();
    target_pose_.pose.orientation.y = q.y();
    target_pose_.pose.orientation.z = q.z();
    target_pose_.pose.orientation.w = q.w();
}

void MavKit::setRawTarget(const mavros_msgs::PositionTarget& target, bool apply_default_yaw) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    raw_target_ = target;
    if (apply_default_yaw && has_default_yaw_) {
        raw_target_.yaw = default_yaw_;
        raw_target_.yaw_rate = 0.0;
        raw_target_.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
        raw_target_.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    }
}

void MavKit::SetGlobalRawTarget(const mavros_msgs::GlobalPositionTarget& target,
                               bool apply_default_yaw) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    global_target_ = target;
    if (apply_default_yaw && has_default_yaw_) {
        global_target_.yaw = default_yaw_;
        global_target_.yaw_rate = 0.0;
        global_target_.type_mask &= ~mavros_msgs::GlobalPositionTarget::IGNORE_YAW;
        global_target_.type_mask |= mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
    }
}

void MavKit::setQuinticCurveTarget(const Eigen::Vector3d& position, bool raw_local) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    const ros::Time now = ros::Time::now();
    captureCurveState(now);
    if (curve_raw_local_ != raw_local) {
        const double angle = raw_local ? calib_.offset() : -calib_.offset();
        const auto rotate = [angle](Eigen::Vector3d& v) {
            const double x = v.x(), y = v.y();
            v.x() = x * std::cos(angle) - y * std::sin(angle);
            v.y() = x * std::sin(angle) + y * std::cos(angle);
        };
        rotate(curve_state_.position);
        // RAW_CTRL 中的速度、加速度设点已经是直接发布的 local 值。
        if (!(raw_local && setpoint_mode_ == SetpointMode::RAW_CTRL)) {
            rotate(curve_state_.velocity);
            rotate(curve_state_.acceleration);
        }
    }
    curve_raw_local_ = raw_local;
    setQuinticCurveTargetLocked(position, now);
}

void MavKit::setQuinticCurveTargetLocked(const Eigen::Vector3d& position, const ros::Time& now) {
    velocity_transition_ = false;

    double v_max, a_max;
    nh_.param<double>("nav/v_max", v_max, 2.5);
    nh_.param<double>("nav/a_max", a_max, 1.2);
    const double distance = (position - curve_state_.position).norm();
    // 沿用 Nav 的时长估算；非零初始导数时并非严格限速求解器。
    const double t_vel = distance * 1.875 / (0.9 * v_max);
    const double t_acc = std::sqrt(distance * 5.774 / a_max);
    curve_time_ = 1.2 * std::max({t_vel, t_acc, 0.5});
    curve_.setBoundaries(
        {curve_state_.position.x(), curve_state_.velocity.x(), curve_state_.acceleration.x()},
        {position.x(), 0.0, 0.0}, curve_time_,
        {curve_state_.position.y(), curve_state_.velocity.y(), curve_state_.acceleration.y()},
        {position.y(), 0.0, 0.0},
        {curve_state_.position.z(), curve_state_.velocity.z(), curve_state_.acceleration.z()},
        {position.z(), 0.0, 0.0});
    curve_start_ = now;
    loadCurveRaw(now);
}

void MavKit::setSetpointMode(SetpointMode mode) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    const ros::Time now = ros::Time::now();
    if (mode == SetpointMode::SLOW_DOWN) {
        // 一次停止只生成一条曲线，重复请求不重置计时。
        if (velocity_transition_) return;
        startStopCurve(now);
        return;
    }
    if (velocity_transition_ && setpoint_mode_ == SetpointMode::SLOW_DOWN &&
        (mode == SetpointMode::HEARTBEAT || mode == SetpointMode::RAW_CTRL ||
         mode == SetpointMode::PID_CTRL)) {
        // 减速阶段不接受视觉速度接管；调用者在停止完成后重新请求。
        return;
    }
    if (mode != SetpointMode::CURVE_CTRL) velocity_transition_ = false;
    if (mode == SetpointMode::PID_CTRL && setpoint_mode_ != SetpointMode::PID_CTRL) {
        pid_x_.reset();
        pid_y_.reset();
        pid_z_.reset();
        pid_last_time_ = now;
    }
    setpoint_mode_ = mode;
}

} // namespace maxt
