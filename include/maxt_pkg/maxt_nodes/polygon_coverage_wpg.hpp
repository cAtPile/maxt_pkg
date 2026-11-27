#ifndef MAXT_POLYGON_COVERAGE_WPG_NODE_HPP
#define MAXT_POLYGON_COVERAGE_WPG_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <maxt_pkg/maxt_nodes/region_waypoints.hpp>

namespace maxt {

// 以第一角点建立切平面，输出经度、纬度航点。
class PolygonCoverageWpG : public BT::SyncActionNode {
public:
    PolygonCoverageWpG(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

};

} // namespace maxt

#endif
