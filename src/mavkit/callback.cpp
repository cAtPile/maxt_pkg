#include <maxt_pkg/mavkit/maxt_mav.hpp>
#include <cmath>

namespace maxt {

// --- 回调函数层 (运行在 AsyncSpinner 线程) ---

void MavKit::stateCb(const mavros_msgs::State::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    current_state_ = *msg;
    ROS_DEBUG("MavKit: State updated: %s", current_state_.connected ? "connected" : "disconnected");
}

void MavKit::extendedStateCb(const mavros_msgs::ExtendedState::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    current_extended_state_ = *msg;
}

void MavKit::poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    current_pose_ = *msg;
    has_pose_ = true;
}

void MavKit::twistCb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    current_twist_ = *msg;
}

void MavKit::globalPoseCb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    has_global_fix_ = msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX &&
        std::isfinite(msg->latitude) && std::isfinite(msg->longitude);
    global_pose_.header = msg->header;
    global_pose_.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    global_pose_.latitude = msg->latitude;
    global_pose_.longitude = msg->longitude;
}

void MavKit::globalAltitudeCb(const std_msgs::Float64::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    // 全球相对高度用于设点转换；任务的高度闭环和到达判断使用 local Z。
    has_global_altitude_ = std::isfinite(msg->data);
    global_pose_.altitude = msg->data;
}

} // namespace maxt
