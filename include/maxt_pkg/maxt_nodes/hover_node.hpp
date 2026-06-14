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
    double start_hover_yaw_{0.0};
};

} // namespace maxt

#endif
