#include <maxt_pkg/mavkit/maxt_mav.hpp>

namespace maxt {

void MavKit::setPIDTarget(const Eigen::Vector3d& position, bool raw_local) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    double kp, ki, kd;
    nh_.param<double>("nav/pid_kp", kp, 1.0);
    nh_.param<double>("nav/pid_ki", ki, 0.05);
    nh_.param<double>("nav/pid_kd", kd, 0.0);
    for (auto* pid : {&pid_x_, &pid_y_, &pid_z_}) {
        pid->reset();
        pid->setGains(kp, ki, kd);
        pid->setOutputClamp(0.5);
        pid->setIntegralClamp(0.3);
    }

    pid_raw_local_ = raw_local;
    pid_last_time_ = ros::Time::now();
    pid_raw_target_ = mavros_msgs::PositionTarget();
    pid_raw_target_.header.frame_id = "map";
    pid_raw_target_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // 沿用 Nav 的位置 + PID 速度修正及零加速度设点。
    pid_raw_target_.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    pid_raw_target_.position.x = position.x();
    pid_raw_target_.position.y = position.y();
    pid_raw_target_.position.z = position.z();
}

void MavKit::loadPIDRaw(const ros::Time& now) {
    auto pose = current_pose_;
    if (!pid_raw_local_) calib_.untransPose(pose);
    const double dt = (now - pid_last_time_).toSec();
    pid_last_time_ = now;
    // ROS 时间回退时清除历史积分和微分，避免沿用上一时间段状态。
    if (dt < 0.0) {
        pid_x_.reset();
        pid_y_.reset();
        pid_z_.reset();
    }
    auto& target = pid_raw_target_;
    target.header.stamp = now;
    target.velocity.x = pid_x_.update(target.position.x, pose.pose.position.x, dt);
    target.velocity.y = pid_y_.update(target.position.y, pose.pose.position.y, dt);
    target.velocity.z = pid_z_.update(target.position.z, pose.pose.position.z, dt);
    // 默认航向已经是 MAVROS local 航向，不参与工程坐标旋转。
    target.yaw = default_yaw_;
}

} // namespace maxt
