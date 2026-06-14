#include <maxt_pkg/maxt_mav.hpp>

namespace maxt {

// --- 指令触发接口 (由 BT 节点的 onStart 或 onRunning 调用) ---

void MavKit::requestArm(bool arm) {
    // 逻辑：如果当前状态已经满足，则不再重复调用
    if (isArmed() == arm) return;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;

    // 注意：在重构后的架构中，这里不使用 while 循环等待
    // 也不检查 arming_client_.call(arm_cmd) 的布尔返回值（防止阻塞）
    // 只是“发出请求”，结果由后续的 isArmed() 轮询检查
    if (!arming_client_.call(arm_cmd)) {
        ROS_ERROR_THROTTLE(1.0, "MavKit: Call Arming Service Failed!");
    }
}

void MavKit::requestMode(const std::string& mode) {
    // 逻辑：如果当前模式已是目标模式，则直接返回
    if (getMode() == mode) return;

    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;

    // 同样，这里只管“踢一脚”，不负责“等结果”
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

} // namespace maxt