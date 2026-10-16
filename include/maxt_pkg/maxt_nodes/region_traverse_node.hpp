#ifndef MAXT_REGION_TRAVERSE_NODE_HPP
#define MAXT_REGION_TRAVERSE_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <array>
#include <vector>

namespace maxt {

// 二维航点保持输入坐标系；该节点只规划，不发送飞行指令。
using RegionWaypoints = std::vector<std::array<double, 2>>;

class RegionTraverseNode : public BT::SyncActionNode {
public:
    RegionTraverseNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// 将数组中的单个航点拆为导航节点的标量输入，并推进索引。
class GetRegionWaypointNode : public BT::SyncActionNode {
public:
    GetRegionWaypointNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

} // namespace maxt

#endif
