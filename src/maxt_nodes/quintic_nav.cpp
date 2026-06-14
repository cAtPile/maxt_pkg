#include <maxt_pkg/maxt_nodes/quintic_nav_node.hpp>
#include <cmath>

namespace maxt {

static constexpr uint16_t TYPE_MASK =
    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

static constexpr uint16_t TYPE_MASK_POS_ONLY =
    mavros_msgs::PositionTarget::IGNORE_VX |
    mavros_msgs::PositionTarget::IGNORE_VY |
    mavros_msgs::PositionTarget::IGNORE_VZ |
    mavros_msgs::PositionTarget::IGNORE_AFX |
    mavros_msgs::PositionTarget::IGNORE_AFY |
    mavros_msgs::PositionTarget::IGNORE_AFZ |
    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

static constexpr double T_MIN = 0.5;
static constexpr double DT_MIN = 1e-6;
static constexpr double HOLD_TIMEOUT = 3.0;

BT::PortsList QuinticNavNode::providedPorts() {
    return {
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("z"),
        BT::InputPort<double>("v_max", 2.5, "max velocity (m/s)"),
        BT::InputPort<double>("a_max", 1.2, "max acceleration (m/s^2)"),
        BT::InputPort<double>("tolerance", 0.1, "arrival tolerance (m)"),
        BT::InputPort<double>("timeout", 60.0, "timeout (s)"),
        BT::InputPort<double>("pid_kp", 1.0, "PID P gain for terminal alignment"),
        BT::InputPort<double>("pid_ki", 0.05, "PID I gain for terminal alignment"),
        BT::InputPort<double>("pid_kd", 0.0, "PID D gain for terminal alignment")
    };
}

QuinticNavNode::QuinticNavNode(const std::string& name,
                               const BT::NodeConfiguration& config,
                               MavKit& mav)
    : MavActionNode(name, config, mav)
{
}

BT::NodeStatus QuinticNavNode::onStart() {
    if (!getInput<double>("x", tx_) ||
        !getInput<double>("y", ty_) ||
        !getInput<double>("z", tz_)) {
        ROS_ERROR("[QuinticNav] Missing target coordinates!");
        return BT::NodeStatus::FAILURE;
    }

    getInput<double>("v_max", v_max_);
    getInput<double>("a_max", a_max_);
    getInput<double>("tolerance", tolerance_);
    getInput<double>("timeout", timeout_);
    getInput<double>("pid_kp", pid_kp_);
    getInput<double>("pid_ki", pid_ki_);
    getInput<double>("pid_kd", pid_kd_);

    auto pose = mav_.getCurrentPose();
    double cx = pose.pose.position.x;
    double cy = pose.pose.position.y;
    double cz = pose.pose.position.z;

    double dx = tx_ - cx;
    double dy = ty_ - cy;
    double dz = tz_ - cz;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    double v_design = v_max_ * 0.9;
    double T_vel = (dist * 1.875) / v_design;
    double T_acc = std::sqrt(dist * 5.774 / a_max_);
    T_ = 1.2 * std::max({T_vel, T_acc, T_MIN});

    QuinticCurve::Boundary s{cx, 0, 0}, e{tx_, 0, 0};
    curve_.setBoundaries(s, e, T_,
                         {cy, 0, 0}, {ty_, 0, 0},
                         {cz, 0, 0}, {tz_, 0, 0});

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

    ROS_INFO("[QuinticNav] START: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f) dist=%.2f T=%.2f",
             cx, cy, cz, tx_, ty_, tz_, dist, T_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus QuinticNavNode::onRunning() {
    ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    if (dt <= DT_MIN) return BT::NodeStatus::RUNNING;
    last_time_ = now;

    double elapsed = (now - start_time_).toSec();
    if (elapsed > timeout_) {
        ROS_ERROR("[QuinticNav] Timeout! elapsed=%.1f", elapsed);
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::FAILURE;
    }

    double t = (now - traj_start_).toSec();
    if (t < 0) t = 0;
    if (t > T_) t = T_;

    double px, py, pz, vx, vy, vz, ax, ay, az;
    curve_.evaluate(t, px, py, pz, vx, vy, vz, ax, ay, az);

    // PID 末端对齐：将位置误差转换为速度修正量叠加到前馈
    {
        auto pose = mav_.getCurrentPose();
        double cx = pose.pose.position.x;
        double cy = pose.pose.position.y;
        double cz = pose.pose.position.z;

        vx += pid_x_.update(px, cx, dt);
        vy += pid_y_.update(py, cy, dt);
        vz += pid_z_.update(pz, cz, dt);
    }

    // Arrival check
    if (t >= T_) {
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
            ROS_INFO("[QuinticNav] REACHED: pos_err=%.3f elapsed=%.2fs",
                     err, elapsed);
            mav_.setSetpointMode(SetpointMode::HEARTBEAT);
            return BT::NodeStatus::SUCCESS;
        }

        double hold_elapsed = (now - hold_start_).toSec();
        if (hold_elapsed > HOLD_TIMEOUT) {
            ROS_WARN("[QuinticNav] Hold timeout (%.1fs), err=%.3f. Forcing SUCCESS.",
                     hold_elapsed, err);
            mav_.setSetpointMode(SetpointMode::HEARTBEAT);
            return BT::NodeStatus::SUCCESS;
        }

        mavros_msgs::PositionTarget hold_target;
        hold_target.header.stamp = ros::Time::now();
        hold_target.header.frame_id = "map";
        hold_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        hold_target.type_mask = TYPE_MASK_POS_ONLY;
        hold_target.position.x = tx_;
        hold_target.position.y = ty_;
        hold_target.position.z = tz_;
        mav_.setRawTarget(hold_target);

        ROS_DEBUG( "[QuinticNav] Holding at target, err=%.3f hold=%.1fs",
                          err, hold_elapsed);
    }

    ROS_DEBUG("[QuinticNav] t=%.2f/%.2f | ref=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f)",
        t, T_, px, py, pz, vx, vy, vz);

    mavros_msgs::PositionTarget target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "map";
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.type_mask = TYPE_MASK;
    target.position.x = px;
    target.position.y = py;
    target.position.z = pz;
    target.velocity.x = vx;
    target.velocity.y = vy;
    target.velocity.z = vz;
    target.acceleration_or_force.x = ax;
    target.acceleration_or_force.y = ay;
    target.acceleration_or_force.z = az;
    mav_.setRawTarget(target);
    mav_.setSetpointMode(SetpointMode::RAW_CTRL);

    return BT::NodeStatus::RUNNING;
}

void QuinticNavNode::onHalted() {
    ROS_WARN("[QuinticNav] Halted!");
    mav_.setSetpointMode(SetpointMode::HEARTBEAT);
}

} // namespace maxt