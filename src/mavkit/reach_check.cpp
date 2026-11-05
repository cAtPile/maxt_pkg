#include <maxt_pkg/mavkit/maxt_mav.hpp>
#include <cmath>

namespace maxt {

bool MavKit::ReachAltitudeCheck(double altitude_raw) const {
    // 同时读取原始位置，不应用 Home 偏移或角度标定。
    std::lock_guard<std::mutex> lock(data_mtx_);
    const double altitude_error = current_pose_.pose.position.z - altitude_raw;
    return std::abs(altitude_error) <= vertical_alt_tolerance_;
}

bool MavKit::ReachPoseCheck(geometry_msgs::PoseStamped pose_raw) const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    const auto& position = current_pose_.pose.position;
    const auto& target = pose_raw.pose.position;
    const double dx = position.x - target.x;
    const double dy = position.y - target.y;
    const double dz = position.z - target.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const auto& linear = current_twist_.twist.linear;
    const double speed = std::sqrt(linear.x * linear.x +
                                   linear.y * linear.y +
                                   linear.z * linear.z);
    return distance <= position_pose_tolerance_ && speed <= position_vel_tolerance_;
}

} // namespace maxt
