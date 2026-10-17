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
    const ros::Time now = ros::Time::now();
    if (setpoint_mode_ == SetpointMode::CURVE_CTRL) {
        // 重规划从切换时刻的参考状态开始，保持 p/v/a 连续。
        loadCurveRaw(now);
    } else {
        auto pose = current_pose_;
        calib_.untransPose(pose);
        curve_state_.position = Eigen::Vector3d(pose.pose.position.x,
                                                pose.pose.position.y,
                                                pose.pose.position.z);
        curve_state_.velocity.setZero();
        curve_state_.acceleration.setZero();
    }

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