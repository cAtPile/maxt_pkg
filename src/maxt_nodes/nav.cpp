#include <maxt_pkg/maxt_nodes/nav_node.hpp>
#include <cmath>

namespace maxt {

static constexpr uint16_t TYPE_MASK =
    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

static constexpr double DT_MIN = 1e-6;
static constexpr double HOLD_TIMEOUT = 3.0;

BT::PortsList NavNode::providedPorts() {
    return {
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("z"),
        BT::InputPort<double>("tolerance", 0.1, "arrival tolerance (m)"),
        BT::InputPort<double>("timeout", 60.0, "timeout (s)")
    };
}

NavNode::NavNode(const std::string& name,
                               const BT::NodeConfiguration& config,
                               MavKit& mav, const ros::NodeHandle& nh)
    : MavActionNode(name, config, mav), nav_nh_(nh, "nav")
{
}

BT::NodeStatus NavNode::onStart() {

    //获取参数
    if (!nav_nh_.getParam("pid_kp", pid_kp_) ||
        !nav_nh_.getParam("pid_ki", pid_ki_) ||
        !nav_nh_.getParam("pid_kd", pid_kd_)) {
        ROS_ERROR("[Nav] Missing configuration under %s; load config/nav.yaml.",
                  nav_nh_.getNamespace().c_str());
        return BT::NodeStatus::FAILURE;
    }

    //验证参数
    if (!std::isfinite(pid_kp_) || !std::isfinite(pid_ki_) ||
        !std::isfinite(pid_kd_)) {
        ROS_ERROR("[Nav] Invalid configuration: PID gains must be finite.");
        return BT::NodeStatus::FAILURE;
    }

    //获取bt参数
    if (!getInput<double>("x", tx_) ||
        !getInput<double>("y", ty_) ||
        !getInput<double>("z", tz_)) {
        ROS_ERROR("[Nav] Missing target coordinates!");
        return BT::NodeStatus::FAILURE;
    }
    getInput<double>("tolerance", tolerance_);
    getInput<double>("timeout", timeout_);

    // 曲线由 MavKit 维护，BT 只负责生命周期和末端闭环。
    mav_.setQuinticCurveTarget(Eigen::Vector3d(tx_, ty_, tz_));
    T_ = mav_.getCurveTime();
    mav_.setSetpointMode(SetpointMode::CURVE_CTRL);

    // 初始化三轴 PID，用于末端闭环对齐
    pid_x_.reset(); pid_x_.setGains(pid_kp_, pid_ki_, pid_kd_);
    pid_y_.reset(); pid_y_.setGains(pid_kp_, pid_ki_, pid_kd_);
    pid_z_.reset(); pid_z_.setGains(pid_kp_, pid_ki_, pid_kd_);
    pid_x_.setOutputClamp(0.5); pid_x_.setIntegralClamp(0.3);
    pid_y_.setOutputClamp(0.5); pid_y_.setIntegralClamp(0.3);
    pid_z_.setOutputClamp(0.5); pid_z_.setIntegralClamp(0.3);

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

    double elapsed = (now - start_time_).toSec();
    if (elapsed > timeout_) {
        ROS_ERROR("[Nav] Timeout! elapsed=%.1f", elapsed);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::FAILURE;
    }

    const double t = (now - traj_start_).toSec();
    if (t < T_) {
        return BT::NodeStatus::RUNNING;
    }

    // 曲线结束后检查到达情况，保留 3 秒强制完成逻辑。
    auto pose = mav_.getCurrentPose();
    double err_x = tx_ - pose.pose.position.x;
    double err_y = ty_ - pose.pose.position.y;
    double err_z = tz_ - pose.pose.position.z;
    double err = std::sqrt(err_x*err_x + err_y*err_y + err_z*err_z);

    if (!holding_) {
        holding_ = true;
        hold_start_ = now;
    }

    if (err < tolerance_) {
        ROS_INFO("[Nav] REACHED: pos_err=%.3f elapsed=%.2fs",
                 err, elapsed);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }

    double hold_elapsed = (now - hold_start_).toSec();
    if (hold_elapsed > HOLD_TIMEOUT) {
        ROS_WARN("[Nav] Hold timeout (%.1fs), err=%.3f. Forcing SUCCESS.",
                 hold_elapsed, err);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }

    // 曲线完成后，在目标位置叠加 PID 速度修正进行末端对齐。
    mavros_msgs::PositionTarget target;
    target.header.stamp = now;
    target.header.frame_id = "map";
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.type_mask = TYPE_MASK;
    target.position.x = tx_;
    target.position.y = ty_;
    target.position.z = tz_;
    target.velocity.x = pid_x_.update(tx_, pose.pose.position.x, dt);
    target.velocity.y = pid_y_.update(ty_, pose.pose.position.y, dt);
    target.velocity.z = pid_z_.update(tz_, pose.pose.position.z, dt);
    // 先填充设点再切换模式，避免发布上次 RAW_CTRL 留下的设点。
    mav_.setRawTarget(target);
    mav_.setSetpointMode(SetpointMode::RAW_CTRL);

    return BT::NodeStatus::RUNNING;
}

void NavNode::onHalted() {
    ROS_WARN("[Nav] Halted! Slowing down.");
    mav_.setSetpointMode(SetpointMode::SLOW_DOWN);
}

} // namespace maxt
