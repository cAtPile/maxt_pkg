#include <maxt_pkg/mavkit/maxt_mav.hpp>
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
    if (setpoint_mode_ == SetpointMode::GLOBAL_RAW_CTRL) {
        // 全球位置控制没有本地参考曲线，中断减速以实测速度作为起点。
        auto twist = current_twist_;
        calib_.untransTwist(twist);
        curve_state_.velocity = Eigen::Vector3d(twist.twist.linear.x,
                                                twist.twist.linear.y,
                                                twist.twist.linear.z);
    }
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

void MavKit::startStopCurve(const ros::Time& now) {
    // 从实测位置和速度开始，统一转换到曲线使用的标定坐标系。
    auto pose = current_pose_;
    auto twist = current_twist_;
    calib_.untransPose(pose);
    calib_.untransTwist(twist);
    const Eigen::Vector3d position(pose.pose.position.x, pose.pose.position.y,
                                  pose.pose.position.z);
    const Eigen::Vector3d velocity(twist.twist.linear.x, twist.twist.linear.y,
                                  twist.twist.linear.z);
    // v(t)=v0*(1-3s²+2s³)，峰值减速度为 1.5*|v0|/T。
    // 初末参考加速度为零，终点位移为 v0*T/2。
    curve_time_ = std::max(slow_down_time_, 1.5 * velocity.norm() / stop_acceleration_);
    const Eigen::Vector3d end = position + 0.5 * velocity * curve_time_;
    curve_.setBoundaries(
        {position.x(), velocity.x(), 0.0}, {end.x(), 0.0, 0.0}, curve_time_,
        {position.y(), velocity.y(), 0.0}, {end.y(), 0.0, 0.0},
        {position.z(), velocity.z(), 0.0}, {end.z(), 0.0, 0.0});
    curve_start_ = now;
    stop_stable_since_ = ros::Time();
    velocity_transition_ = true;
    loadCurveRaw(now);
    setpoint_mode_ = SetpointMode::SLOW_DOWN;
    ROS_INFO("[Stop] START: speed=%.3f m/s duration=%.3f s", velocity.norm(), curve_time_);
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
    curve_raw_target_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    curve_raw_target_.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    curve_raw_target_.yaw = default_yaw_;
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
