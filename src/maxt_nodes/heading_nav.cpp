#include <maxt_pkg/maxt_nodes/heading_nav_node.hpp>
#include <cmath>

namespace maxt {

static constexpr uint16_t TYPE_MASK =
    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

static constexpr double DT_MIN = 1e-6;
static constexpr double HOLD_TIMEOUT = 3.0;

BT::PortsList HeadingNavNode::providedPorts() {
    return {
        BT::InputPort<double>("heading_angle", "travel heading in control frame (deg)"),
        BT::InputPort<double>("nav_dis", "horizontal travel distance (m)"),
        BT::InputPort<double>("tolerance", 0.1, "arrival tolerance (m)"),
        BT::InputPort<double>("timeout", 60.0, "timeout (s)")
    };
}

HeadingNavNode::HeadingNavNode(const std::string& name,
                               const BT::NodeConfiguration& config,
                               MavKit& mav, const ros::NodeHandle& nh)
    : MavActionNode(name, config, mav), nav_nh_(nh, "nav")
{
}

BT::NodeStatus HeadingNavNode::onStart() {

    //获取参数
    if (!nav_nh_.getParam("pid_kp", pid_kp_) ||
        !nav_nh_.getParam("pid_ki", pid_ki_) ||
        !nav_nh_.getParam("pid_kd", pid_kd_)) {
        ROS_ERROR("[HeadingNav] Missing configuration under %s; load config/nav.yaml.",
                  nav_nh_.getNamespace().c_str());
        return BT::NodeStatus::FAILURE;
    }

    //验证参数
    if (!std::isfinite(pid_kp_) || !std::isfinite(pid_ki_) ||
        !std::isfinite(pid_kd_)) {
        ROS_ERROR("[HeadingNav] Invalid configuration: PID gains must be finite.");
        return BT::NodeStatus::FAILURE;
    }

    double heading_angle, nav_dis;
    if (!getInput("heading_angle", heading_angle) || !getInput("nav_dis", nav_dis)) {
        ROS_ERROR("[HeadingNav] Missing heading_angle or nav_dis!");
        return BT::NodeStatus::FAILURE;
    }
    getInput("tolerance", tolerance_);
    getInput("timeout", timeout_);

    // 每次启动重新捕获起点；航向相对控制坐标系 +X，逆时针为正。
    // 目标直接使用控制坐标，Z 保持启动时的高度。
    // 输入使用角度制，三角函数计算前转换为弧度。
    const double heading_rad = heading_angle * M_PI / 180.0;
    const auto pose = mav_.getCurrentPose();
    tx_ = pose.pose.position.x + nav_dis * std::cos(heading_rad);
    ty_ = pose.pose.position.y + nav_dis * std::sin(heading_rad);
    tz_ = pose.pose.position.z;

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

    ROS_INFO("[HeadingNav] START: target=(%.2f,%.2f,%.2f) T=%.2f", tx_, ty_, tz_, T_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HeadingNavNode::onRunning() {
    ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    if (dt <= DT_MIN) return BT::NodeStatus::RUNNING;
    last_time_ = now;

    double elapsed = (now - start_time_).toSec();
    if (elapsed > timeout_) {
        ROS_ERROR("[HeadingNav] Timeout! elapsed=%.1f", elapsed);
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
        ROS_INFO("[HeadingNav] REACHED: pos=(%.3f, %.3f, %.3f) pos_err=%.3f elapsed=%.2fs",
                 pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                 err, elapsed);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::SUCCESS;
    }

    double hold_elapsed = (now - hold_start_).toSec();
    if (hold_elapsed > HOLD_TIMEOUT) {
        ROS_WARN("[HeadingNav] Hold timeout (%.1fs), err=%.3f. Forcing SUCCESS.",
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

void HeadingNavNode::onHalted() {
    ROS_WARN("[HeadingNav] Halted! Slowing down.");
    mav_.setSetpointMode(SetpointMode::SLOW_DOWN);
}

} // namespace maxt
