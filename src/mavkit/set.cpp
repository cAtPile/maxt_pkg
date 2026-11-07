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

void MavKit::SetGlobalRawTarget(const mavros_msgs::GlobalPositionTarget& target) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    global_target_ = target;
    if (has_default_yaw_) {
        global_target_.yaw = default_yaw_;
        global_target_.yaw_rate = 0.0;
        global_target_.type_mask &= ~mavros_msgs::GlobalPositionTarget::IGNORE_YAW;
        global_target_.type_mask |= mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
    }
}

void MavKit::setQuinticCurveTarget(const Eigen::Vector3d& position) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    setQuinticCurveTargetLocked(position, ros::Time::now());
}

void MavKit::setQuinticCurveTargetLocked(const Eigen::Vector3d& position, const ros::Time& now) {
    captureCurveState(now);
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
        // 重复停止请求不重置减速时间。
        if (velocity_transition_ && transition_end_mode_ == SetpointMode::HEARTBEAT) return;
        startVelocityTransition(Eigen::Vector3d::Zero(), now);
        transition_end_mode_ = SetpointMode::HEARTBEAT;
        return;
    }
    if (velocity_transition_ && setpoint_mode_ == SetpointMode::SLOW_DOWN) {
        if (mode == SetpointMode::HEARTBEAT) {
            // Guidance 初始化/等待反馈也会请求心跳，不能跳过尚未完成的减速。
            if (transition_end_mode_ != SetpointMode::HEARTBEAT) {
                startVelocityTransition(Eigen::Vector3d::Zero(), now);
                transition_end_mode_ = SetpointMode::HEARTBEAT;
            }
            return;
        }
        if (mode == SetpointMode::RAW_CTRL) {
            Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
            if (!(raw_target_.type_mask & mavros_msgs::PositionTarget::IGNORE_VX)) velocity.x() = raw_target_.velocity.x;
            if (!(raw_target_.type_mask & mavros_msgs::PositionTarget::IGNORE_VY)) velocity.y() = raw_target_.velocity.y;
            if (!(raw_target_.type_mask & mavros_msgs::PositionTarget::IGNORE_VZ)) velocity.z() = raw_target_.velocity.z;
            // 相同速度目标不重新计时，确保重复发布也能完成过渡。
            if (transition_end_mode_ == SetpointMode::RAW_CTRL &&
                (velocity - transition_velocity_).squaredNorm() < 1e-12) return;
            startVelocityTransition(velocity, now);//TODO：这是什么？
            transition_end_mode_ = SetpointMode::RAW_CTRL;
            return;
        }
        if (mode == SetpointMode::CONTROL) {
            setQuinticCurveTargetLocked(Eigen::Vector3d(target_pose_.pose.position.x,
                                                       target_pose_.pose.position.y,
                                                       target_pose_.pose.position.z), now);
            setpoint_mode_ = SetpointMode::CURVE_CTRL;
            return;
        }
    }
    if (mode != SetpointMode::CURVE_CTRL) velocity_transition_ = false;
    setpoint_mode_ = mode;
}

} // namespace maxt
