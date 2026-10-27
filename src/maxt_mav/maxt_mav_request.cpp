#include <maxt_pkg/maxt_mav.hpp>
#include <algorithm>
#include <cmath>

namespace maxt {

// --- 指令触发接口 (由 BT 节点的 onStart 或 onRunning 调用) ---

void MavKit::requestArm(bool arm) {
    if (isArmed() == arm) return;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;

    if (!arming_client_.call(arm_cmd)) {
        ROS_ERROR_THROTTLE(1.0, "MavKit: Call Arming Service Failed!");
    }
}

void MavKit::requestMode(const std::string& mode) {

    if (getMode() == mode) return;

    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;

    if (!mode_client_.call(mode_cmd)) {
        ROS_ERROR_THROTTLE(1.0, "MavKit: Call SetMode Service Failed!");
    }
}

// --- 辅助管理接口 ---

void MavKit::updateHomePose() {
    std::lock_guard<std::mutex> lock(data_mtx_);
    if (has_pose_) {
        home_pose_ = current_pose_;
        ROS_INFO_THROTTLE(1.0, "MavKit: Home Pose Updated to [%.2f, %.2f, %.2f]",
                 home_pose_.pose.position.x, 
                 home_pose_.pose.position.y, 
                 home_pose_.pose.position.z);
    } else {
        ROS_WARN_THROTTLE(1.0, "MavKit: Cannot update Home, no pose data yet!");
    }
}

geometry_msgs::PoseStamped MavKit::getHomePose() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return home_pose_;
}

void MavKit::setRawTarget(const mavros_msgs::PositionTarget& target) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    raw_target_ = target;
}

void MavKit::setQuinticCurveTarget(const Eigen::Vector3d& position) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    setQuinticCurveTargetLocked(position, ros::Time::now());
}

void MavKit::captureCurveState(const ros::Time& now) {
    if (setpoint_mode_ == SetpointMode::CURVE_CTRL ||
        setpoint_mode_ == SetpointMode::SLOW_DOWN) {
        loadCurveRaw(now);
        return;
    }
    auto pose = current_pose_;
    calib_.untransPose(pose);
    curve_state_.position = Eigen::Vector3d(pose.pose.position.x,
                                            pose.pose.position.y,
                                            pose.pose.position.z);
    curve_state_.velocity.setZero();
    curve_state_.acceleration.setZero();
    if (setpoint_mode_ == SetpointMode::RAW_CTRL) {
        // 原始设点也在世界坐标系中；忽略的速度轴没有参考速度，保持零。
        const uint16_t velocity_masks[] = {mavros_msgs::PositionTarget::IGNORE_VX,
                                          mavros_msgs::PositionTarget::IGNORE_VY,
                                          mavros_msgs::PositionTarget::IGNORE_VZ};
        const uint16_t acceleration_masks[] = {mavros_msgs::PositionTarget::IGNORE_AFX,
                                              mavros_msgs::PositionTarget::IGNORE_AFY,
                                              mavros_msgs::PositionTarget::IGNORE_AFZ};
        const double velocity[] = {raw_target_.velocity.x, raw_target_.velocity.y, raw_target_.velocity.z};
        const double acceleration[] = {raw_target_.acceleration_or_force.x,
                                       raw_target_.acceleration_or_force.y,
                                       raw_target_.acceleration_or_force.z};
        for (int i = 0; i < 3; ++i) {
            if (!(raw_target_.type_mask & velocity_masks[i])) curve_state_.velocity[i] = velocity[i];
            if (!(raw_target_.type_mask & acceleration_masks[i])) curve_state_.acceleration[i] = acceleration[i];
        }
    }
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

void MavKit::startVelocityTransition(const Eigen::Vector3d& velocity, const ros::Time& now) {
    captureCurveState(now);
    curve_time_ = slow_down_time_;
    transition_velocity_ = velocity;
    // 终点位置由参考速度、加速度及过渡时长推导，不强制停在中断位置。
    // 该位置对应满足 v(0)、a(0)、v(T)、a(T)=0 的三次速度曲线积分。
    const Eigen::Vector3d position = curve_state_.position +
        0.5 * (curve_state_.velocity + velocity) * curve_time_ +
        curve_state_.acceleration * curve_time_ * curve_time_ / 12.0;
    curve_.setBoundaries(
        {curve_state_.position.x(), curve_state_.velocity.x(), curve_state_.acceleration.x()},
        {position.x(), velocity.x(), 0.0}, curve_time_,
        {curve_state_.position.y(), curve_state_.velocity.y(), curve_state_.acceleration.y()},
        {position.y(), velocity.y(), 0.0},
        {curve_state_.position.z(), curve_state_.velocity.z(), curve_state_.acceleration.z()},
        {position.z(), velocity.z(), 0.0});
    curve_start_ = now;
    velocity_transition_ = true;
    loadCurveRaw(now);
    setpoint_mode_ = SetpointMode::SLOW_DOWN;
}

double MavKit::getCurveTime() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return curve_time_;
}

void MavKit::loadCurveRaw(const ros::Time& now) {
    const double t = std::max(0.0, std::min((now - curve_start_).toSec(), curve_time_));
    curve_.evaluate(t,
                    curve_state_.position.x(), curve_state_.position.y(), curve_state_.position.z(),
                    curve_state_.velocity.x(), curve_state_.velocity.y(), curve_state_.velocity.z(),
                    curve_state_.acceleration.x(), curve_state_.acceleration.y(), curve_state_.acceleration.z());
    curve_raw_target_.header.stamp = now;
    curve_raw_target_.header.frame_id = "map";
    curve_raw_target_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    curve_raw_target_.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    curve_raw_target_.position.x = curve_state_.position.x();
    curve_raw_target_.position.y = curve_state_.position.y();
    curve_raw_target_.position.z = curve_state_.position.z();
    curve_raw_target_.velocity.x = curve_state_.velocity.x();
    curve_raw_target_.velocity.y = curve_state_.velocity.y();
    curve_raw_target_.velocity.z = curve_state_.velocity.z();
    curve_raw_target_.acceleration_or_force.x = curve_state_.acceleration.x();
    curve_raw_target_.acceleration_or_force.y = curve_state_.acceleration.y();
    curve_raw_target_.acceleration_or_force.z = curve_state_.acceleration.z();
}

} // namespace maxt