#ifndef MAXT_NODES_QUINTIC_NAV_HPP
#define MAXT_NODES_QUINTIC_NAV_HPP

#include <maxt_pkg/maxt_nodes.hpp>
#include <maxt_pkg/quintic_curve.hpp>
#include <maxt_pkg/pid_controller.hpp>

namespace maxt {

class QuinticNavNode : public MavActionNode {
public:
    QuinticNavNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    double tx_{0.0}, ty_{0.0}, tz_{0.0};
    double v_max_{2.5}, a_max_{1.2};
    double tolerance_{0.1};
    double timeout_{60.0};
    double pid_kp_{1.0}, pid_ki_{0.05}, pid_kd_{0.0};

    QuinticCurve curve_;
    double T_{1.0};
    ros::Time traj_start_;
    ros::Time start_time_;
    ros::Time last_time_;
    ros::Time hold_start_;
    bool holding_{false};

    PIDController pid_x_, pid_y_, pid_z_;
};

} // namespace maxt

#endif
