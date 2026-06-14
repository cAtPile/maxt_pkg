#ifndef MAXT_NODES_LAND_HPP
#define MAXT_NODES_LAND_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class LandNode : public MavActionNode {
public:
    LandNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    ros::Time start_time_;
};

} // namespace maxt

#endif
