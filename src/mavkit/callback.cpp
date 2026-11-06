#include <maxt_pkg/mavkit/maxt_mav.hpp>

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

} // namespace maxt
