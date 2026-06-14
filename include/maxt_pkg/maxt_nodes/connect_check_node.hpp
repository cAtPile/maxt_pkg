#ifndef MAXT_NODES_CONNECT_CHECK_HPP
#define MAXT_NODES_CONNECT_CHECK_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class ConnectCheckNode : public MavActionNode {
public:
    ConnectCheckNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    ros::WallTime start_time_;
    double timeout_;
    double map_x_{0}, map_y_{0};
    double fact_x_{0}, fact_y_{0};
};

} // namespace maxt

#endif
