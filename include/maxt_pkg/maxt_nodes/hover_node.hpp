#ifndef MAXT_NODES_HOVER_HPP
#define MAXT_NODES_HOVER_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class HoverNode : public MavActionNode {
public:
    HoverNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    double hover_duration_{1.0};
    ros::Time start_time_;
    geometry_msgs::PoseStamped start_hover_pose_;
};

class Px4HoldNode : public MavActionNode {
public:
    using MavActionNode::MavActionNode;

    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    ros::SteadyTime started_;
    ros::SteadyTime next_request_;
};

} // namespace maxt

#endif
