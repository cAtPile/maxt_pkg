#include <maxt_pkg/maxt_nodes/re_heading_node.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <cmath>

namespace maxt {
namespace {
double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
double yawOf(const geometry_msgs::PoseStamped& pose) {
    const auto& q = pose.pose.orientation;
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
    return yaw;
}
}

ReHeadingNode::ReHeadingNode(const std::string& name, const BT::NodeConfiguration& config,
                           MavKit& mav, const ros::NodeHandle& nh)
    : MavActionNode(name, config, mav), nh_(nh) {}

BT::PortsList ReHeadingNode::providedPorts() {
    return {BT::InputPort<double>("timeout", 60.0, "timeout (s)")};
}

BT::NodeStatus ReHeadingNode::onStart() {
    double angle, rate, acceleration, tolerance_deg, kp, ki, kd;
    nh_.param<double>("nav/azimuth_angle", angle, 0.0);
    nh_.param<double>("nav/hold_timeout", hold_timeout_, 3.0);
    nh_.param<double>("re_heading/yaw_rate_max", rate, 0.5);
    nh_.param<double>("re_heading/yaw_acc_max", acceleration, 0.5);
    nh_.param<double>("re_heading/tolerance_deg", tolerance_deg, 2.0);
    getInput("timeout", timeout_);
    if (!nh_.getParam("nav/pid_kp", kp) || !nh_.getParam("nav/pid_ki", ki) ||
        !nh_.getParam("nav/pid_kd", kd)) {
        ROS_ERROR("[ReHeading] Missing nav PID configuration; load config/nav.yaml.");
        return BT::NodeStatus::FAILURE;
    }
    if (!std::isfinite(angle) || !std::isfinite(rate) || rate <= 0.0 ||
        !std::isfinite(acceleration) || acceleration <= 0.0 ||
        !std::isfinite(tolerance_deg) || tolerance_deg <= 0.0 ||
        !std::isfinite(timeout_) || timeout_ <= 0.0 ||
        !std::isfinite(hold_timeout_) || hold_timeout_ < 0.0 ||
        !std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd)) {
        ROS_ERROR("[ReHeading] Invalid configuration.");
        return BT::NodeStatus::FAILURE;
    }
    const auto pose = mav_.GetRawPose();
    const auto& q = pose.pose.orientation;
    const double norm2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(norm2) || norm2 < 1e-12 ||
        !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
        !std::isfinite(pose.pose.position.z)) {
        ROS_ERROR("[ReHeading] Invalid raw pose.");
        return BT::NodeStatus::FAILURE;
    }
    position_ = pose.pose.position;
    last_yaw_ = measured_yaw_ = yawOf(pose);
    target_yaw_ = wrap(std::remainder(angle, 360.0) * M_PI / 180.0);
    const double delta = wrap(target_yaw_ - last_yaw_);
    unwrapped_target_ = measured_yaw_ + delta;
    tolerance_ = tolerance_deg * M_PI / 180.0;
    duration_ = 1.2 * std::max({1.875 * std::abs(delta) / (0.9 * rate),
                              std::sqrt(5.774 * std::abs(delta) / acceleration), 0.5});
    QuinticCurve::solveAxis(measured_yaw_, 0.0, 0.0, unwrapped_target_, 0.0, 0.0,
                            duration_, coefficients_);
    start_time_ = last_time_ = ros::Time::now();
    holding_ = false;
    publish(last_yaw_, 0.0, false);
    mav_.setSetpointMode(SetpointMode::RAW_CTRL);
    ROS_INFO("[ReHeading] START: yaw=%.2f deg target=%.2f deg T=%.2fs",
             last_yaw_ * 180.0 / M_PI, target_yaw_ * 180.0 / M_PI, duration_);
    return BT::NodeStatus::RUNNING;
}

void ReHeadingNode::publish(double yaw, double yaw_rate, bool rate_control) {
    mavros_msgs::PositionTarget target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "map";
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ;
    if (rate_control) target.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
    target.position = position_;
    target.yaw = wrap(yaw);
    target.yaw_rate = yaw_rate;
    mav_.setRawTarget(target, false);
}

BT::NodeStatus ReHeadingNode::onRunning() {
    const auto now = ros::Time::now();
    const double dt = (now - last_time_).toSec();
    if (dt <= 1e-6) return BT::NodeStatus::RUNNING;
    last_time_ = now;
    const double elapsed = (now - start_time_).toSec();
    const double yaw = yawOf(mav_.GetRawPose());
    if (elapsed > timeout_) {
        ROS_ERROR("[ReHeading] Timeout! elapsed=%.2fs", elapsed);
        mav_.setDefaultYaw(yaw);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::FAILURE;
    }
    if (elapsed < duration_) {
        publish(QuinticCurve::evalPos(coefficients_, elapsed),
                QuinticCurve::evalVel(coefficients_, elapsed), false);
        return BT::NodeStatus::RUNNING;
    }
    if (!holding_) {
        holding_ = true;
        hold_start_ = now;
        mav_.setPIDYawTarget(position_, target_yaw_);
        mav_.setSetpointMode(SetpointMode::PID_CTRL);
    }
    const double error = wrap(target_yaw_ - yaw);
    if (std::abs(error) < tolerance_ || (now - hold_start_).toSec() > hold_timeout_) {
        if (std::abs(error) < tolerance_) {
            ROS_INFO("[ReHeading] REACHED: yaw_err=%.2f deg", error * 180.0 / M_PI);
        } else {
            ROS_WARN("[ReHeading] Hold timeout, yaw_err=%.2f deg. Forcing SUCCESS.",
                     error * 180.0 / M_PI);
        }
        mav_.setDefaultYaw(target_yaw_);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void ReHeadingNode::onHalted() {
    mav_.setDefaultYaw(yawOf(mav_.GetRawPose()));
    mav_.setSetpointMode(SetpointMode::SLOW_DOWN);
}
} // namespace maxt
