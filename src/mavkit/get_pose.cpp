#include <maxt_pkg/maxt_mav.hpp>

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

} // namespace maxt
