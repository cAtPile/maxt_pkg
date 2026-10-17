#ifndef MAXT_NODES_NAV_HPP
#define MAXT_NODES_NAV_HPP

#include <maxt_pkg/maxt_nodes.hpp>
#include <maxt_pkg/pid_controller.hpp>

namespace maxt {

class NavNode : public MavActionNode {
public:
    NavNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav, const ros::NodeHandle& nh);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    ros::NodeHandle nav_nh_;
    double tx_{0.0}, ty_{0.0}, tz_{0.0};
    double tolerance_{0.1};
    double timeout_{60.0};
    double pid_kp_{1.0}, pid_ki_{0.05}, pid_kd_{0.0};

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
