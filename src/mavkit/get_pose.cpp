#include <maxt_pkg/mavkit/maxt_mav.hpp>
#include <cmath>

namespace maxt {
namespace pose_tools {

geometry_msgs::PoseStamped offsetHome(const geometry_msgs::PoseStamped& pose,
                                      const geometry_msgs::PoseStamped& home) {
    auto out = pose;
    out.pose.position.x -= home.pose.position.x;
    out.pose.position.y -= home.pose.position.y;
    out.pose.position.z -= home.pose.position.z;
    return out;
}

} // namespace pose_tools

geometry_msgs::PoseStamped MavKit::GetRawPose() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return current_pose_;
}

void MavKit::RecordRawHomePose() {
    std::lock_guard<std::mutex> lock(data_mtx_);
    // 完整保留 MAVROS 原始消息，不应用 Home 偏移或角度标定。
    raw_home_pose_ = current_pose_;
}

geometry_msgs::PoseStamped MavKit::GetRawHomePose() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return raw_home_pose_;
}

geometry_msgs::PoseStamped MavKit::GetHomeOffsetPose() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return pose_tools::offsetHome(current_pose_, home_pose_);
}

geometry_msgs::PoseStamped MavKit::GetHomeCalibratePose() const {
    // 当前点、Home 和标定参数在同一把锁内读取，避免混用不同快照。
    std::lock_guard<std::mutex> lock(data_mtx_);
    auto out = pose_tools::offsetHome(current_pose_, home_pose_);
    calib_.untransPose(out);
    return out;
}

geometry_msgs::PoseStamped MavKit::GetCalibratePose() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    auto out = current_pose_;
    calib_.untransPose(out);
    return out;
}

//TODO:针对getHomePose 我获得到的 home_pose_ 是golbal的数据home_pose_应该不用经过untrans
geometry_msgs::PoseStamped MavKit::getHomePose() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    geometry_msgs::PoseStamped out = home_pose_;
    calib_.untransPose(out);
    return out;
}

mavros_msgs::GlobalPositionTarget MavKit::GetGlobalPose() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return global_pose_;
}

bool MavKit::hasGlobalPose() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return has_global_fix_ && has_global_altitude_ && has_pose_;
}

bool MavKit::hasFreshNavigationData(bool global, double max_age) const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    const auto now = ros::Time::now();
    const auto fresh = [&](const ros::Time& stamp) {
        const double age = (now-stamp).toSec();
        return !stamp.isZero() && age >= -0.05 && age <= max_age;
    };
    const auto& p = current_pose_.pose.position;
    const auto& v = current_twist_.twist.linear;
    const auto& q = current_pose_.pose.orientation;
    const double norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if (!has_pose_ || !fresh(current_pose_.header.stamp) || !fresh(current_twist_.header.stamp) ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
        !std::isfinite(norm) || norm < 1e-12) return false;
    return !global || (has_global_fix_ && has_global_altitude_ && fresh(global_pose_.header.stamp) &&
        !global_altitude_received_.isZero() &&
        (ros::WallTime::now()-global_altitude_received_).toSec() <= max_age);
}

} // namespace maxt
