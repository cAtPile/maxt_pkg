#ifndef MAXT_NODES_XNAV_HPP
#define MAXT_NODES_XNAV_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class XNavigationNode : public MavActionNode {
public:
    XNavigationNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    double target_x_{0.0};
    double vx_{1.0};
    double tolerance_{0.3};
    double timeout_{30.0};

    double hold_y_{0.0}, hold_z_{0.0}, hold_yaw_{0.0};
    ros::Time start_time_;


};

} // namespace maxt

#endif
