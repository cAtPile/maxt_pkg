#include <maxt_pkg/maxt_nodes/nav/nav_azimuth_node.hpp>
#include <algorithm>
#include <cmath>

namespace maxt {

namespace {
constexpr uint16_t TYPE_MASK = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
constexpr double DT_MIN = 1e-6;

mavros_msgs::PositionTarget makeTarget(const ros::Time& now) {
    mavros_msgs::PositionTarget target;
    target.header.stamp = now;
    target.header.frame_id = "map";
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.type_mask = TYPE_MASK;
    return target;
}
} // namespace

NavAzimuth::NavAzimuth(const std::string& name,
    const BT::NodeConfiguration& config, MavKit& mav, const ros::NodeHandle& nh)
    : MavActionNode(name, config, mav), nh_(nh) {}

BT::PortsList NavAzimuth::providedPorts() {
    return {
        BT::OutputPort<double>("breakpoint_x"),
        BT::OutputPort<double>("breakpoint_y"),
        BT::OutputPort<double>("breakpoint_z"),
        BT::OutputPort<bool>("breakpoint_valid"),
            BT::InputPort<double>("nav_n"),
            BT::InputPort<double>("nav_t"),
            BT::InputPort<double>("nav_a"),
            BT::InputPort<double>("bp_x_raw"),
            BT::InputPort<double>("bp_y_raw"),
            BT::InputPort<double>("bp_z_raw"),
            BT::InputPort<double>("timeout", 60.0, "timeout (s)")};
}

BT::NodeStatus NavAzimuth::onStart() {
    nh_.param<double>("nav/hold_timeout", hold_timeout_, 3.0);
    double n, t, a, bx, by, bz;
    if (!getInput("nav_n", n) || !getInput("nav_t", t) || !getInput("nav_a", a) ||
        !getInput("bp_x_raw", bx) || !getInput("bp_y_raw", by) ||
        !getInput("bp_z_raw", bz)) {
        ROS_ERROR("[NavAzimuth] Missing displacement or raw base point.");
        return BT::NodeStatus::FAILURE;
    }
    getInput("timeout", timeout_);

    double angle, kp, ki, kd, v_max, a_max;
    nh_.param<double>("nav/azimuth_angle", angle, 0.0);
    nh_.param<double>("reach_check/position_pose_tolerance", tolerance_, 0.1);
    if (!nh_.getParam("nav/pid_kp", kp) || !nh_.getParam("nav/pid_ki", ki) ||
        !nh_.getParam("nav/pid_kd", kd)) {
        ROS_ERROR("[NavAzimuth] Missing PID configuration; load config/nav.yaml.");
        return BT::NodeStatus::FAILURE;
    }
    nh_.param<double>("nav/v_max", v_max, 2.5);
    nh_.param<double>("nav/a_max", a_max, 1.2);

    // 正角度从原始 +X 转向 +Y；横向正方向为左侧，不改变机头 yaw。
    const double radians = angle * M_PI / 180.0;
    tx_ = bx + n * std::cos(radians) - t * std::sin(radians);
    ty_ = by + n * std::sin(radians) + t * std::cos(radians);
    tz_ = bz + a;

    // 曲线在原始坐标系内采样，RAW_CTRL 不再应用 Home 或标定变换。
    const auto pose = mav_.GetRawPose();
    const auto& p = pose.pose.position;
    const double distance = std::sqrt(std::pow(tx_ - p.x, 2) +
        std::pow(ty_ - p.y, 2) + std::pow(tz_ - p.z, 2));
    const double t_vel = distance * 1.875 / (0.9 * v_max);
    const double t_acc = std::sqrt(distance * 5.774 / a_max);
    duration_ = 1.2 * std::max({t_vel, t_acc, 0.5});
    curve_.setWaypoints(p.x, p.y, p.z, tx_, ty_, tz_, duration_);
    start_time_ = ros::Time::now();
    last_time_ = start_time_;
    holding_ = false;
    publishCurve(0.0, start_time_);
    mav_.setSetpointMode(SetpointMode::RAW_CTRL);
    ROS_INFO("[NavAzimuth] START: raw target=(%.2f,%.2f,%.2f) T=%.2f",
             tx_, ty_, tz_, duration_);
    return BT::NodeStatus::RUNNING;
}

void NavAzimuth::publishCurve(double t, const ros::Time& now) {
    auto target = makeTarget(now);
    curve_.evaluate(t, target.position.x, target.position.y, target.position.z,
                    target.velocity.x, target.velocity.y, target.velocity.z,
                    target.acceleration_or_force.x, target.acceleration_or_force.y,
                    target.acceleration_or_force.z);
    mav_.setRawTarget(target);
}

BT::NodeStatus NavAzimuth::onRunning() {
    const auto now = ros::Time::now();
    const double dt = (now - last_time_).toSec();
    if (dt <= DT_MIN) return BT::NodeStatus::RUNNING;
    last_time_ = now;
    const double elapsed = (now - start_time_).toSec();
    if (elapsed > timeout_) {
        ROS_ERROR("[NavAzimuth] Timeout! elapsed=%.1f", elapsed);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::FAILURE;
    }
    if (elapsed < duration_) {
        publishCurve(elapsed, now);
        return BT::NodeStatus::RUNNING;
    }

    const auto pose = mav_.GetRawPose();
    const auto& p = pose.pose.position;
    const double error = std::sqrt(std::pow(tx_ - p.x, 2) +
        std::pow(ty_ - p.y, 2) + std::pow(tz_ - p.z, 2));
    if (!holding_) {
        holding_ = true;
        hold_start_ = now;
        // 仅在进入末端阶段时设置，避免每个 tick 清除积分。
        mav_.setPIDTarget(Eigen::Vector3d(tx_, ty_, tz_), true);
        mav_.setSetpointMode(SetpointMode::PID_CTRL);
    }
    if (error < tolerance_) {
        ROS_INFO("[NavAzimuth] REACHED: pos_err=%.3f elapsed=%.2fs", error, elapsed);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }
    if ((now - hold_start_).toSec() > hold_timeout_) {
        ROS_WARN("[NavAzimuth] Hold timeout (%.1fs), err=%.3f. Forcing SUCCESS.",
                 (now - hold_start_).toSec(), error);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}



void NavAzimuth::onHalted() {
    recordBreakpointAndSlowDown();
}

} // namespace maxt
