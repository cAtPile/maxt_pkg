#ifndef MAXT_NODES_GOTO_HPP
#define MAXT_NODES_GOTO_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class GoToNode : public MavActionNode {
public:
    GoToNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    double tx_{0.0}, ty_{0.0}, tz_{0.0};
    double hold_yaw_{0.0};
    double step_{0.05};
    double timeout_{30.0};
    double tolerance_{0.3};
    ros::Time start_time_;
};

} // namespace maxt

#endif
