#include <maxt_pkg/maxt_nodes/round_node.hpp>
#include <maxt_pkg/quintic_curve.hpp>
#include <cmath>

namespace maxt {

static constexpr uint16_t ROUND_TYPE_MASK =
    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

BT::PortsList RoundNode::providedPorts() {
    return {
        BT::InputPort<double>("center_x"),
        BT::InputPort<double>("center_y"),
        BT::InputPort<double>("center_z"),
        BT::InputPort<double>("radius", 0.0, "radius(m), auto-calculated if 0"),
        BT::InputPort<double>("angular_speed", 30.0, "angular speed (deg/s)"),
        BT::InputPort<double>("total_angle", 360.0, "total rotation angle (deg)"),
        BT::InputPort<double>("tolerance", 0.3, "arrival tolerance (m)"),
        BT::InputPort<double>("timeout", 60.0, "global timeout (s)"),
        BT::InputPort<double>("ramp_time", 2.0, "accel/decel ramp time (s)")
    };
}

RoundNode::RoundNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav)
    , desired_angle_(0.0)
    , angular_speed_(30.0)
    , total_angle_(360.0)
    , final_target_set_(false)
{
}

BT::NodeStatus RoundNode::onStart() {
    if (!getInput<double>("center_x", cx_) ||
        !getInput<double>("center_y", cy_) ||
        !getInput<double>("center_z", cz_)) {
        ROS_ERROR("RoundNode: Center coordinates missing!");
        return BT::NodeStatus::FAILURE;
    }

    auto current_pose = mav_.getCurrentPose();
    double curr_x = current_pose.pose.position.x;
    double curr_y = current_pose.pose.position.y;

    double dx = curr_x - cx_;
    double dy = curr_y - cy_;

    double input_r = 0.0;
    getInput<double>("radius", input_r);
    radius_ = (input_r > 0.1) ? input_r : std::sqrt(dx*dx + dy*dy);
    start_phi_ = std::atan2(dy, dx);

    getInput<double>("angular_speed", angular_speed_);
    getInput<double>("total_angle", total_angle_);
    getInput<double>("timeout", timeout_);
    getInput<double>("ramp_time", ramp_time_);

    last_time_ = ros::Time::now();
    start_time_ = last_time_;
    ramp_start_time_ = last_time_;
    hold_yaw_ = mav_.get_current_yaw();

    desired_angle_ = 0.0;
    last_reported_milestone_ = 0.0;
    final_target_set_ = false;

    double omega_rad = angular_speed_ * M_PI / 180.0;

    // Solve acceleration quintic: 0 → ω_target
    QuinticCurve::solveAxis(0.0, 0.0, 0.0,
                            0.5 * omega_rad * ramp_time_,
                            omega_rad, 0.0,
                            ramp_time_, accel_coeffs_);

    // Solve deceleration quintic: ω_target → 0
    QuinticCurve::solveAxis(0.0, omega_rad, 0.0,
                            0.5 * omega_rad * ramp_time_,
                            0.0, 0.0,
                            ramp_time_, decel_coeffs_);

    ramp_angle_deg_ = QuinticCurve::evalPos(accel_coeffs_, ramp_time_) * 180.0 / M_PI;
    const_speed_angle_deg_ = 0.0;

    // If total angle is too short for both ramps, skip to decel from start
    if (total_angle_ <= 2.0 * ramp_angle_deg_) {
        ramp_phase_ = RampPhase::ACCEL;  // accel will transition directly
    } else {
        ramp_phase_ = RampPhase::ACCEL;
    }

    // Publish initial setpoint (ω=0, α=0, at current position)
    double theta = start_phi_;
    mavros_msgs::PositionTarget target;
    target.header.frame_id = "map";
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.type_mask = ROUND_TYPE_MASK;
    target.position.x = cx_ + radius_ * std::cos(theta);
    target.position.y = cy_ + radius_ * std::sin(theta);
    target.position.z = cz_;
    target.velocity.x = 0.0;
    target.velocity.y = 0.0;
    target.velocity.z = 0.0;
    target.acceleration_or_force.x = 0.0;
    target.acceleration_or_force.y = 0.0;
    target.acceleration_or_force.z = 0.0;
    target.yaw = hold_yaw_;
    mav_.setRawTarget(target);
    mav_.setSetpointMode(SetpointMode::RAW_CTRL);

    ROS_INFO("RoundNode: PVA orbit start. R=%.2f phi0=%.1f deg omega_tgt=%.1f deg/s "
             "total=%.1f deg ramp=%.1fs ramp_angle=%.1f deg yaw=%.1f",
             radius_, start_phi_ * 180.0 / M_PI, angular_speed_,
             total_angle_, ramp_time_, ramp_angle_deg_, hold_yaw_ * 180.0 / M_PI);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RoundNode::onRunning() {
    ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    if (dt <= 0.0) dt = 0.01;
    last_time_ = now;

    if ((now - start_time_).toSec() > timeout_) {
        ROS_ERROR("RoundNode: Timeout!");
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
        return BT::NodeStatus::FAILURE;
    }

    double omega_rad = angular_speed_ * M_PI / 180.0;
    double omega_curr = 0.0;
    double alpha_curr = 0.0;

    switch (ramp_phase_) {
    case RampPhase::ACCEL: {
        double t = (now - ramp_start_time_).toSec();
        bool ramp_done = (t >= ramp_time_);
        if (ramp_done) t = ramp_time_;

        desired_angle_ = QuinticCurve::evalPos(accel_coeffs_, t) * 180.0 / M_PI;
        omega_curr = QuinticCurve::evalVel(accel_coeffs_, t);
        alpha_curr = QuinticCurve::evalAcc(accel_coeffs_, t);

        if (ramp_done) {
            double remaining = total_angle_ - ramp_angle_deg_;
            if (remaining <= ramp_angle_deg_) {
                ramp_phase_ = RampPhase::DECEL;
                decel_start_time_ = now;
                const_speed_angle_deg_ = 0.0;
            } else {
                ramp_phase_ = RampPhase::CONSTANT;
            }
        }
        break;
    }

    case RampPhase::CONSTANT: {
        double t_const = (now - ramp_start_time_).toSec() - ramp_time_;
        desired_angle_ = ramp_angle_deg_ + angular_speed_ * t_const;
        omega_curr = omega_rad;
        alpha_curr = 0.0;

        double remaining = total_angle_ - desired_angle_;
        if (remaining <= ramp_angle_deg_) {
            ramp_phase_ = RampPhase::DECEL;
            decel_start_time_ = now;
            const_speed_angle_deg_ = angular_speed_ * t_const;
        }
        break;
    }
    
    case RampPhase::DECEL: {
        double t = (now - decel_start_time_).toSec();
        if (t >= ramp_time_) {
            ramp_phase_ = RampPhase::FINAL;
            desired_angle_ = total_angle_;
            omega_curr = 0.0;
            alpha_curr = 0.0;
            final_target_set_ = true;
        } else {
            desired_angle_ = ramp_angle_deg_ + const_speed_angle_deg_
                           + QuinticCurve::evalPos(decel_coeffs_, t) * 180.0 / M_PI;
            omega_curr = QuinticCurve::evalVel(decel_coeffs_, t);
            alpha_curr = QuinticCurve::evalAcc(decel_coeffs_, t);
        }
        break;
    }
    case RampPhase::FINAL: {
        desired_angle_ = total_angle_;
        omega_curr = 0.0;
        alpha_curr = 0.0;
        break;
    }
    }

    // Milestone reporting
    {
        int current_lap = static_cast<int>(desired_angle_ / 360.0);
        int last_lap = static_cast<int>(last_reported_milestone_ / 360.0);
        if (current_lap > last_lap) {
            last_reported_milestone_ = current_lap * 360.0;
            ROS_INFO("RoundNode: %.0f/%.0f deg [%s]",
                     last_reported_milestone_, total_angle_,
                     ramp_phase_ == RampPhase::ACCEL  ? "accel"  :
                     ramp_phase_ == RampPhase::DECEL  ? "decel"  :
                     ramp_phase_ == RampPhase::CONSTANT ? "const" : "final");
        }
    }

    // Compute circular PVA (clockwise)
    double theta = start_phi_ - desired_angle_ * M_PI / 180.0;
    double ct = std::cos(theta);
    double st = std::sin(theta);

    double px = cx_ + radius_ * ct;
    double py = cy_ + radius_ * st;

    double vx =  radius_ * omega_curr * st;
    double vy = -radius_ * omega_curr * ct;

    double ax =  radius_ * alpha_curr * st - radius_ * omega_curr * omega_curr * ct;
    double ay = -radius_ * alpha_curr * ct - radius_ * omega_curr * omega_curr * st;

    mavros_msgs::PositionTarget target;
    target.header.frame_id = "map";
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.type_mask = ROUND_TYPE_MASK;
    target.position.x = px;
    target.position.y = py;
    target.position.z = cz_;
    target.velocity.x = vx;
    target.velocity.y = vy;
    target.velocity.z = 0.0;
    target.acceleration_or_force.x = ax;
    target.acceleration_or_force.y = ay;
    target.acceleration_or_force.z = 0.0;
    target.yaw = hold_yaw_;
    mav_.setRawTarget(target);

    // Convergence check (FINAL phase only)
    if (ramp_phase_ == RampPhase::FINAL) {
        double tolerance = 0.3;
        getInput<double>("tolerance", tolerance);
        auto current_pose = mav_.getCurrentPose();
        double ex = px - current_pose.pose.position.x;
        double ey = py - current_pose.pose.position.y;
        double ez = cz_ - current_pose.pose.position.z;
        double error = std::sqrt(ex*ex + ey*ey + ez*ez);
        if (error < tolerance) {
            ROS_INFO("RoundNode: Task Finished.");
            mav_.setSetpointMode(SetpointMode::HEARTBEAT);
            return BT::NodeStatus::SUCCESS;
        }
    }

    return BT::NodeStatus::RUNNING;
}

void RoundNode::onHalted() {
    mav_.setSetpointMode(SetpointMode::HEARTBEAT);
}

} // namespace maxt
