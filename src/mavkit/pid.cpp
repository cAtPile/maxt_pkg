#include <maxt_pkg/mavkit/maxt_mav.hpp>
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

void MavKit::resetPIDControllers() {
    for (auto* pid : {&pid_x_, &pid_y_, &pid_z_, &pid_yaw_}) pid->reset();
    // 航向连续展开，避免跨越 +/-pi 时微分跳变。
    pid_last_yaw_ = pid_measured_yaw_ = yawOf(current_pose_);
}

void MavKit::configurePIDControllers() {
    double kp, ki, kd, yaw_rate;
    nh_.param<double>("nav/pid_kp", kp, 1.0);
    nh_.param<double>("nav/pid_ki", ki, 0.05);
    nh_.param<double>("nav/pid_kd", kd, 0.0);
    nh_.param<double>("re_heading/yaw_rate_max", yaw_rate, 0.5);
    for (auto* pid : {&pid_x_, &pid_y_, &pid_z_, &pid_yaw_}) {
        pid->setGains(kp, ki, kd);
        pid->setOutputClamp(0.5);
        pid->setIntegralClamp(0.3);
    }
    pid_yaw_.setOutputClamp(yaw_rate);
    resetPIDControllers();
    pid_last_time_ = ros::Time::now();
}

void MavKit::setPIDTarget(const Eigen::Vector3d& position, bool raw_local) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    configurePIDControllers();
    pid_target_kind_ = PIDTargetKind::POSITION;
    pid_raw_local_ = raw_local;
    pid_raw_target_ = mavros_msgs::PositionTarget();
    pid_raw_target_.header.frame_id = "map";
    pid_raw_target_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // 沿用 Nav 的位置 + PID 速度修正及零加速度设点。
    pid_raw_target_.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    pid_raw_target_.position.x = position.x();
    pid_raw_target_.position.y = position.y();
    pid_raw_target_.position.z = position.z();
}

void MavKit::setPIDYawTarget(const geometry_msgs::Point& position, double yaw) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    configurePIDControllers();
    pid_target_kind_ = PIDTargetKind::YAW;
    pid_raw_local_ = true;
    pid_target_yaw_ = yaw;
    pid_raw_target_ = mavros_msgs::PositionTarget();
    auto& target = pid_raw_target_;
    target.header.frame_id = "map";
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.position = position;
    using T = mavros_msgs::PositionTarget;
    target.type_mask = T::IGNORE_VX | T::IGNORE_VY | T::IGNORE_VZ |
        T::IGNORE_AFX | T::IGNORE_AFY | T::IGNORE_AFZ | T::IGNORE_YAW;
}

void MavKit::setGlobalPIDTarget(const GlobalNavCurve& curve,
                               const std::array<double, 6>& yaw_coefficients,
                               double yaw_duration, const ros::Time& start) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    configurePIDControllers();
    pid_target_kind_ = PIDTargetKind::GLOBAL;
    pid_global_curve_ = curve;
    pid_yaw_coefficients_ = yaw_coefficients;
    pid_yaw_duration_ = yaw_duration;
    pid_global_start_ = start;
    pid_target_yaw_ = wrap(QuinticCurve::evalPos(yaw_coefficients.data(), yaw_duration));
}

double MavKit::updatePIDYaw(double target, double dt) {
    const double yaw = yawOf(current_pose_);
    pid_measured_yaw_ += wrap(yaw - pid_last_yaw_);
    pid_last_yaw_ = yaw;
    const double error = wrap(target - pid_measured_yaw_);
    return pid_yaw_.update(pid_measured_yaw_ + error, pid_measured_yaw_, dt);
}

void MavKit::loadPIDRaw(const ros::Time& now) {
    const double dt = (now - pid_last_time_).toSec();
    pid_last_time_ = now;
    if (dt < 0.0) resetPIDControllers();

    if (pid_target_kind_ == PIDTargetKind::GLOBAL) {
        const double elapsed = std::max(0.0, (now - pid_global_start_).toSec());
        auto& target = pid_global_raw_target_;
        target = pid_global_curve_.sample(elapsed);
        target.header.stamp = now;
        if (elapsed >= pid_global_curve_.duration()) {
            // 全球误差始终用 WGS-84 测地距离计算；保持原有相对高度设点。
            double distance, azimuth, end_azimuth;
            GeographicLib::Geodesic::WGS84().Inverse(
                global_pose_.latitude, global_pose_.longitude,
                target.latitude, target.longitude, distance, azimuth, end_azimuth);
            const double heading = azimuth * M_PI / 180.0;
            target.velocity.x = pid_x_.update(0.0, -distance * std::sin(heading), dt);
            target.velocity.y = pid_y_.update(0.0, -distance * std::cos(heading), dt);
        }
        using T = mavros_msgs::GlobalPositionTarget;
        target.type_mask &= ~(T::IGNORE_YAW | T::IGNORE_YAW_RATE);
        if (elapsed < pid_yaw_duration_) {
            target.yaw = wrap(QuinticCurve::evalPos(pid_yaw_coefficients_.data(), elapsed));
            target.yaw_rate = QuinticCurve::evalVel(pid_yaw_coefficients_.data(), elapsed);
        } else {
            target.type_mask |= T::IGNORE_YAW;
            target.yaw = pid_target_yaw_;
            target.yaw_rate = updatePIDYaw(pid_target_yaw_, dt);
        }
        return;
    }

    auto& target = pid_raw_target_;
    target.header.stamp = now;
    if (pid_target_kind_ == PIDTargetKind::YAW) {
        target.yaw = pid_target_yaw_;
        target.yaw_rate = updatePIDYaw(pid_target_yaw_, dt);
        return;
    }
    auto pose = current_pose_;
    if (!pid_raw_local_) calib_.untransPose(pose);
    target.velocity.x = pid_x_.update(target.position.x, pose.pose.position.x, dt);
    target.velocity.y = pid_y_.update(target.position.y, pose.pose.position.y, dt);
    target.velocity.z = pid_z_.update(target.position.z, pose.pose.position.z, dt);
    // 默认航向已经是 MAVROS local 航向，不参与工程坐标旋转。
    target.yaw = default_yaw_;
}

} // namespace maxt
