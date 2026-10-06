#include <maxt_pkg/numeric_sim.hpp>

#include <limits>

namespace maxt {

/**
 * @brief 发布当前仿真时间和所有 MavKit 订阅的 MAVROS 状态话题。
 */
void NumericSim::publishState() {
    // /use_sim_time=true 时，ROS 节点的 ros::Time::now() 由 /clock 驱动。
    // 因此必须每步先发 clock，再发携带同一时刻 stamp 的位姿/速度。
    rosgraph_msgs::Clock clock;
    clock.clock = ros::Time(sim_time_);
    clock_pub_.publish(clock);

    const ros::Time stamp(sim_time_);
    pose_.header.stamp = stamp;
    // 本仿真的所有位置都处于固定 map 坐标系，未模拟 TF 树或机体坐标系。
    pose_.header.frame_id = "map";
    twist_.header.stamp = stamp;
    state_.connected = config_.connected;
    state_pub_.publish(state_);
    extended_state_pub_.publish(extended_state_);
    pose_pub_.publish(pose_);
    twist_pub_.publish(twist_);
}

/**
 * @brief 返回当前命令的指定位置目标；无位置命令时返回 NaN，便于 CSV 区分。
 */
double NumericSim::targetValue(std::size_t axis) const {
    // NaN 比填 0 更适合表示“不适用”：分析 CSV 时不会把缺失的目标
    // 误解为“命令飞到原点”。注意 RAW 中被 mask 忽略的原始值仍会被记录。
    if (command_kind_ == CommandKind::POSE) {
        return axis == 0 ? pose_target_.pose.position.x
                         : (axis == 1 ? pose_target_.pose.position.y
                                      : pose_target_.pose.position.z);
    }
    if (command_kind_ == CommandKind::RAW) {
        return axis == 0 ? raw_target_.position.x
                         : (axis == 1 ? raw_target_.position.y : raw_target_.position.z);
    }
    return std::numeric_limits<double>::quiet_NaN();
}

/**
 * @brief 返回当前 raw 命令的指定速度目标；普通位置命令和空命令返回 NaN。
 */
double NumericSim::targetVelocity(std::size_t axis) const {
    if (command_kind_ != CommandKind::RAW) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return axis == 0 ? raw_target_.velocity.x
                     : (axis == 1 ? raw_target_.velocity.y : raw_target_.velocity.z);
}

/**
 * @brief 返回当前 raw 命令的指定加速度目标；该值只记录，不参与理想跟随模型。
 */
double NumericSim::targetAcceleration(std::size_t axis) const {
    if (command_kind_ != CommandKind::RAW) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return axis == 0 ? raw_target_.acceleration_or_force.x
                     : (axis == 1 ? raw_target_.acceleration_or_force.y
                                  : raw_target_.acceleration_or_force.z);
}

/**
 * @brief 将内部枚举转换为稳定的 CSV/日志命令类别文本。
 */
std::string NumericSim::commandKindName() const {
    switch (command_kind_) {
    case CommandKind::NONE:
        return "none";
    case CommandKind::POSE:
        return "pose";
    case CommandKind::RAW:
        return "raw";
    }
    return "unknown";
}

/**
 * @brief 在 CSV 启用时追加当前仿真状态和最新 setpoint 的一行快照。
 */
void NumericSim::writeCsv() {
    if (!csv_.is_open()) {
        return;
    }
    // 每次 step 一行，所以采样周期等于 dt。文件流在析构时关闭，
    // 由标准库完成最后的 flush；不在每行强制 flush，以减少 I/O 开销。
    const bool is_raw = command_kind_ == CommandKind::RAW;
    csv_ << sim_time_ << ',' << pose_.pose.position.x << ',' << pose_.pose.position.y << ','
         << pose_.pose.position.z << ',' << twist_.twist.linear.x << ','
         << twist_.twist.linear.y << ',' << twist_.twist.linear.z << ',' << yaw_ << ','
         << (state_.connected ? 1 : 0) << ',' << (state_.armed ? 1 : 0) << ','
         << state_.mode << ','
         << (extended_state_.landed_state ==
                     mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND
                 ? 1
                 : 0)
         << ',' << commandKindName() << ',' << targetValue(0) << ',' << targetValue(1) << ','
         << targetValue(2) << ',' << targetVelocity(0) << ',' << targetVelocity(1) << ','
         << targetVelocity(2) << ',' << targetAcceleration(0) << ','
         << targetAcceleration(1) << ',' << targetAcceleration(2) << ','
         << (is_raw ? raw_target_.yaw : yaw_) << ','
         << (is_raw ? raw_target_.yaw_rate : 0.0) << ','
         << (is_raw ? raw_target_.type_mask : 0) << '\n';
}

/**
 * @brief 在 report_period 指定的仿真时间间隔打印一次紧凑状态摘要。
 */
void NumericSim::reportIfDue() {
    if (config_.report_period <= 0.0 ||
        (last_report_time_ >= 0.0 &&
         sim_time_ - last_report_time_ < config_.report_period)) {
        return;
    }
    last_report_time_ = sim_time_;
    ROS_INFO("NumericSim: t=%.1f pose=(%.2f, %.2f, %.2f) yaw=%.2f mode=%s armed=%d landed=%d cmd=%s",
             sim_time_, pose_.pose.position.x, pose_.pose.position.y,
             pose_.pose.position.z, yaw_, state_.mode.c_str(), state_.armed,
             extended_state_.landed_state ==
                 mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND,
             commandKindName().c_str());
}

}  // namespace maxt
