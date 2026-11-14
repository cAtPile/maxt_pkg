#include <maxt_pkg/maxt_nodes/nav/nav_raw_node.hpp>
#include <cmath>

namespace maxt {

static constexpr double DT_MIN = 1e-6;

BT::PortsList NavRawNode::providedPorts() {
    return {
        BT::OutputPort<double>("breakpoint_x"),
        BT::OutputPort<double>("breakpoint_y"),
        BT::OutputPort<double>("breakpoint_z"),
        BT::OutputPort<bool>("breakpoint_valid"),
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("z"),
        BT::InputPort<double>("timeout", 60.0, "timeout (s)")
    };
}

NavRawNode::NavRawNode(const std::string& name,
                               const BT::NodeConfiguration& config,
                               MavKit& mav, const ros::NodeHandle& nh)
    : MavActionNode(name, config, mav), nav_nh_(nh, "nav")
{
    nh.param<double>("reach_check/position_pose_tolerance", tolerance_, 0.1);
}

BT::NodeStatus NavRawNode::onStart() {
    waiting_for_stop_ = false;
    nav_nh_.param<double>("hold_timeout", hold_timeout_, 3.0);

    //获取参数
    if (!nav_nh_.getParam("pid_kp", pid_kp_) ||
        !nav_nh_.getParam("pid_ki", pid_ki_) ||
        !nav_nh_.getParam("pid_kd", pid_kd_)) {
        ROS_ERROR("[NavRaw] Missing configuration under %s; load config/nav.yaml.",
                  nav_nh_.getNamespace().c_str());
        return BT::NodeStatus::FAILURE;
    }

    //验证参数
    if (!std::isfinite(pid_kp_) || !std::isfinite(pid_ki_) ||
        !std::isfinite(pid_kd_)) {
        ROS_ERROR("[NavRaw] Invalid configuration: PID gains must be finite.");
        return BT::NodeStatus::FAILURE;
    }

    //获取bt参数
    if (!getInput<double>("x", tx_) ||
        !getInput<double>("y", ty_) ||
        !getInput<double>("z", tz_)) {
        ROS_ERROR("[NavRaw] Missing target coordinates!");
        return BT::NodeStatus::FAILURE;
    }
    getInput<double>("timeout", timeout_);

    waiting_for_stop_ = mav_.isSlowingDown();
    if (waiting_for_stop_) {
        ROS_INFO("[NavRaw] Waiting for slowdown to complete before navigation.");
        return BT::NodeStatus::RUNNING;
    }
    return startNavigation();
}

BT::NodeStatus NavRawNode::startNavigation() {
    // x/y/z 直接使用 MAVROS 原始 local 坐标，不叠加 Home 或标定旋转。
    // 曲线由 MavKit 维护，BT 只负责生命周期和末端闭环。
    mav_.setQuinticCurveTarget(Eigen::Vector3d(tx_, ty_, tz_), true);
    T_ = mav_.getCurveTime();
    mav_.setSetpointMode(SetpointMode::CURVE_CTRL);

    start_time_ = ros::Time::now();
    traj_start_ = start_time_;
    last_time_ = start_time_;
    holding_ = false;

    ROS_INFO("[NavRaw] START: target=(%.2f,%.2f,%.2f) T=%.2f", tx_, ty_, tz_, T_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavRawNode::onRunning() {
    if (waiting_for_stop_) {
        if (mav_.isSlowingDown()) return BT::NodeStatus::RUNNING;
        waiting_for_stop_ = false;
        return startNavigation();
    }
    ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    if (dt <= DT_MIN) return BT::NodeStatus::RUNNING;
    last_time_ = now;

    double elapsed = (now - start_time_).toSec();
    if (elapsed > timeout_) {
        ROS_ERROR("[NavRaw] Timeout! elapsed=%.1f", elapsed);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::FAILURE;
    }

    const double t = (now - traj_start_).toSec();
    if (t < T_) {
        return BT::NodeStatus::RUNNING;
    }

    // 曲线结束后检查到达情况，按配置的末端对齐超时强制完成。
    auto pose = mav_.GetRawPose();
    double err_x = tx_ - pose.pose.position.x;
    double err_y = ty_ - pose.pose.position.y;
    double err_z = tz_ - pose.pose.position.z;
    double err = std::sqrt(err_x*err_x + err_y*err_y + err_z*err_z);

    if (!holding_) {
        holding_ = true;
        hold_start_ = now;
        // 仅在进入末端阶段时设置，避免每个 tick 清除积分。
        mav_.setPIDTarget(Eigen::Vector3d(tx_, ty_, tz_), true);
        mav_.setSetpointMode(SetpointMode::PID_CTRL);
    }

    if (err < tolerance_) {
        ROS_INFO("[NavRaw] REACHED: pos=(%.3f, %.3f, %.3f) pos_err=%.3f elapsed=%.2fs",
                 pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                 err, elapsed);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }

    double hold_elapsed = (now - hold_start_).toSec();
    if (hold_elapsed > hold_timeout_) {
        ROS_WARN("[NavRaw] Hold timeout (%.1fs), err=%.3f. Forcing SUCCESS.",
                 hold_elapsed, err);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}



void NavRawNode::onHalted() {
    recordBreakpointAndSlowDown();
}

} // namespace maxt
