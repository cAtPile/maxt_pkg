#include <maxt_pkg/maxt_nodes/nav/nav_node.hpp>
#include <cmath>

namespace maxt {

void MavActionNode::recordBreakpointAndSlowDown() {
    const auto pose = mav_.GetRawPose();
    const auto& p = pose.pose.position;
    const bool valid = !pose.header.stamp.isZero() &&
        std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    setOutput("breakpoint_valid", false);
    if (valid) {
        setOutput("breakpoint_x", p.x);
        setOutput("breakpoint_y", p.y);
        setOutput("breakpoint_z", p.z);
        setOutput("breakpoint_valid", true);
        ROS_WARN("[%s] Halted: breakpoint=(%.3f, %.3f, %.3f). Slowing down.",
                 name().c_str(), p.x, p.y, p.z);
    } else {
        ROS_WARN("[%s] Halted without valid pose; breakpoint unavailable. Slowing down.",
                 name().c_str());
    }
    mav_.setSetpointMode(SetpointMode::SLOW_DOWN);
}

static constexpr double DT_MIN = 1e-6;

BT::PortsList NavNode::providedPorts() {
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

NavNode::NavNode(const std::string& name,
                               const BT::NodeConfiguration& config,
                               MavKit& mav, const ros::NodeHandle& nh)
    : MavActionNode(name, config, mav), nav_nh_(nh, "nav")
{
    nh.param<double>("reach_check/position_pose_tolerance", tolerance_, 0.1);
}

BT::NodeStatus NavNode::onStart() {
    nav_nh_.param<double>("hold_timeout", hold_timeout_, 3.0);

    //获取bt参数
    if (!getInput<double>("x", tx_) ||
        !getInput<double>("y", ty_) ||
        !getInput<double>("z", tz_)) {
        ROS_ERROR("[Nav] Missing target coordinates!");
        return BT::NodeStatus::FAILURE;
    }
    getInput<double>("timeout", timeout_);

    // 输入为相对 home 的偏移，统一转换为控制坐标。
    const auto home = mav_.getHomePose();
    tx_ += home.pose.position.x;
    ty_ += home.pose.position.y;
    tz_ += home.pose.position.z;

    // 曲线由 MavKit 维护，BT 只负责生命周期和末端闭环。
    mav_.setQuinticCurveTarget(Eigen::Vector3d(tx_, ty_, tz_));
    T_ = mav_.getCurveTime();
    mav_.setSetpointMode(SetpointMode::CURVE_CTRL);

    start_time_ = ros::Time::now();
    traj_start_ = start_time_;
    last_time_ = start_time_;
    holding_ = false;

    ROS_INFO("[Nav] START: target=(%.2f,%.2f,%.2f) T=%.2f", tx_, ty_, tz_, T_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavNode::onRunning() {
    ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    if (dt <= DT_MIN) return BT::NodeStatus::RUNNING;
    last_time_ = now;

    //超时失败
    double elapsed = (now - start_time_).toSec();
    if (elapsed > timeout_) {
        ROS_ERROR("[Nav] Timeout! elapsed=%.1f", elapsed);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::FAILURE;
    }

    //曲线控制状态
    const double t = (now - traj_start_).toSec();
    if (t < T_) {
        return BT::NodeStatus::RUNNING;
    }

    //TODO：到达检查可以用mavkit
    // 曲线结束后检查到达情况，按配置的末端对齐超时强制完成。
    auto pose = mav_.GetCalibratePose();
    double err_x = tx_ - pose.pose.position.x;
    double err_y = ty_ - pose.pose.position.y;
    double err_z = tz_ - pose.pose.position.z;
    double err = std::sqrt(err_x*err_x + err_y*err_y + err_z*err_z);

    //进入PID末端对齐
    if (!holding_) {
        holding_ = true;
        hold_start_ = now;
        mav_.setPIDTarget(Eigen::Vector3d(tx_, ty_, tz_));
        mav_.setSetpointMode(SetpointMode::PID_CTRL);
    }

    //达标成功
    if (err < tolerance_) {
        ROS_INFO("[Nav] REACHED: pos=(%.3f, %.3f, %.3f) pos_err=%.3f elapsed=%.2fs",
                 pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                 err, elapsed);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }

    //末端强制成功
    double hold_elapsed = (now - hold_start_).toSec();
    if (hold_elapsed > hold_timeout_) {
        ROS_WARN("[Nav] Hold timeout (%.1fs), err=%.3f. Forcing SUCCESS.",
                 hold_elapsed, err);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}



void NavNode::onHalted() {
    recordBreakpointAndSlowDown();
}

} // namespace maxt
