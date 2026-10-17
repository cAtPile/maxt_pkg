#ifndef MAXT_REGION_TRAVERSE_NODE_HPP
#define MAXT_REGION_TRAVERSE_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <maxt_pkg/maxt_nodes/region_waypoints.hpp>

namespace maxt {

// 二维航点保持输入坐标系；该节点只规划，不发送飞行指令。
class RegionTraverseNode : public BT::SyncActionNode {
public:
    RegionTraverseNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

} // namespace maxt

#endif
