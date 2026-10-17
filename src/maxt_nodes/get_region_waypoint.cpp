#include <maxt_pkg/maxt_nodes/get_region_waypoint_node.hpp>

namespace maxt {

BT::PortsList GetRegionWaypointNode::providedPorts() {
    return {
        BT::InputPort<RegionWaypoints>("waypoints"),
        BT::InputPort<int>("index"),
        BT::OutputPort<double>("x"),
        BT::OutputPort<double>("y"),
        BT::OutputPort<int>("next_index")
    };
}

BT::NodeStatus GetRegionWaypointNode::tick() {
    RegionWaypoints waypoints;
    int index;
    if (!getInput("waypoints", waypoints) || !getInput("index", index) ||
        index < 0 || static_cast<std::size_t>(index) >= waypoints.size()) {
        return BT::NodeStatus::FAILURE;
    }
    setOutput("x", waypoints[index][0]);
    setOutput("y", waypoints[index][1]);
    setOutput("next_index", index + 1);
    return BT::NodeStatus::SUCCESS;
}

} // namespace maxt
