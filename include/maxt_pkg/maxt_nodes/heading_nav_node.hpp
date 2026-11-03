#ifndef MAXT_NODES_HEADING_NAV_HPP
#define MAXT_NODES_HEADING_NAV_HPP

#include <maxt_pkg/maxt_nodes/nav_node.hpp>
#include <memory>

namespace maxt {

// 将固定航向和距离转换为航点，导航生命周期交给 NavNode。
class HeadingNavNode : public MavActionNode {
public:
    HeadingNavNode(const std::string& name, const BT::NodeConfiguration& config,
                   MavKit& mav, const ros::NodeHandle& nh);

    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    ros::NodeHandle nh_;
    std::unique_ptr<NavNode> nav_;
};

} // namespace maxt

#endif
