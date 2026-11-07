#include <maxt_pkg/mavkit/maxt_mav.hpp>
#include <cmath>
#include <GeographicLib/Geodesic.hpp>

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

bool MavKit::ReachGlobalCheck(const mavros_msgs::GlobalPositionTarget& target,
                              double altitude_raw) const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    if (!has_global_fix_ || !has_pose_) return false;

    // WGS-84 椭球上的最短测地距离，单位为米，避免球体近似误差。
    double distance;
    GeographicLib::Geodesic::WGS84().Inverse(
        global_pose_.latitude, global_pose_.longitude,
        target.latitude, target.longitude, distance);
    const auto& velocity = current_twist_.twist.linear;
    const double speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y +
                                   velocity.z * velocity.z);
    return distance <= global_position_tolerance_ &&
        std::abs(current_pose_.pose.position.z - altitude_raw) <= global_altitude_tolerance_ &&
        speed <= position_vel_tolerance_;
}

} // namespace maxt
