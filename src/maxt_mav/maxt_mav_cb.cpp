#include <maxt_pkg/mavkit/maxt_mav.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace maxt {

// --- 线程安全的数据查询接口 (运行在 BT 线程) ---

bool MavKit::isConnected() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return current_state_.connected;
}

bool MavKit::isArmed() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return current_state_.armed;
}

bool MavKit::isLand() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return current_extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
}

std::string MavKit::getMode() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return current_state_.mode;
}

geometry_msgs::TwistStamped MavKit::getCurrentTwist() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    geometry_msgs::TwistStamped out = current_twist_;
    calib_.untransTwist(out);
    return out;
}

double MavKit::get_current_yaw() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    tf2::Quaternion q(current_pose_.pose.orientation.x,
                      current_pose_.pose.orientation.y,
                      current_pose_.pose.orientation.z,
                      current_pose_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

bool MavKit::isReached(double x, double y, double z, double tolerance) const {
    geometry_msgs::PoseStamped curr = GetCalibratePose();  // world coords
    double dx = curr.pose.position.x - x;
    double dy = curr.pose.position.y - y;
    double dz = curr.pose.position.z - z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    return distance < tolerance;
}

} // namespace maxt
