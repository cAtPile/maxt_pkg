#ifndef MAXT_NODES_ROUND_HPP
#define MAXT_NODES_ROUND_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class RoundNode : public MavActionNode {
public:
    RoundNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    enum class RampPhase { ACCEL, CONSTANT, DECEL, FINAL };

    double cx_, cy_, cz_;
    double tx_, ty_, tz_;
    double radius_;
    double start_phi_;
    double angular_speed_{30.0};
    double desired_angle_{0.0};
    double total_angle_{360.0};
    bool final_target_set_{false};
    double timeout_;
    double hold_yaw_;
    double last_reported_milestone_{0.0};
    ros::Time start_time_;
    ros::Time last_time_;

    double ramp_time_{2.0};
    ros::Time ramp_start_time_;
    double accel_coeffs_[6]{};
    double decel_coeffs_[6]{};
    double ramp_angle_deg_{0.0};
    RampPhase ramp_phase_{RampPhase::ACCEL};
    ros::Time decel_start_time_;
    double const_speed_angle_deg_{0.0};
};

} // namespace maxt

#endif
