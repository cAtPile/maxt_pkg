#ifndef MAXT_NODES_WAITSTEP_HPP
#define MAXT_NODES_WAITSTEP_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class WaitStepNode : public MavActionNode {
public:
    WaitStepNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    double wait_duration_;
    ros::Time start_time_;
};

} // namespace maxt

#endif
