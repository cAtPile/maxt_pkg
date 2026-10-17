#ifndef MAXT_GET_REGION_WAYPOINT_NODE_HPP
#define MAXT_GET_REGION_WAYPOINT_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <maxt_pkg/maxt_nodes/region_waypoints.hpp>

namespace maxt {

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
