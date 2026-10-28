#include <maxt_pkg/maxt_nodes/traverse_path_generator_node.hpp>
#include <cmath>
#include <algorithm>

namespace maxt {

BT::PortsList TraversePathGeneratorNode::providedPorts() {
    return {
        BT::InputPort<double>("origin_x"),
        BT::InputPort<double>("origin_y"),
        BT::InputPort<double>("length_x"),
        BT::InputPort<double>("length_y"),
        BT::InputPort<double>("coverage_step"),
        BT::OutputPort<RegionWaypoints>("waypoints", "Ordered XY waypoint array"),
        BT::OutputPort<int>("waypoint_count", "Number of waypoints")
    };
}

BT::NodeStatus TraversePathGeneratorNode::tick() {
    double origin_x, origin_y, length_x, length_y, coverage_step;
    if (!getInput("origin_x", origin_x) || !getInput("origin_y", origin_y) ||
        !getInput("length_x", length_x) || !getInput("length_y", length_y) ||
        !getInput("coverage_step", coverage_step) ||
        !std::isfinite(origin_x) || !std::isfinite(origin_y) ||
        !std::isfinite(length_x) || !std::isfinite(length_y) ||
        !std::isfinite(coverage_step) || coverage_step <= 0.0 ||
        length_x == 0.0 || length_y == 0.0 ||
        !std::isfinite(origin_x + length_x) || !std::isfinite(origin_y + length_y)) {
        return BT::NodeStatus::FAILURE;
    }

    // 航带沿有符号 X 边长往返，沿有符号 Y 边长推进。
    const double width = std::abs(length_y);

    // 保持相邻平行轨迹的间距为 coverage_step，余量对称留在两侧。
    const double lanes_needed = std::ceil(width / coverage_step);
    constexpr int max_lanes = 100000;

    if (!std::isfinite(lanes_needed) || width <= 0.0 || lanes_needed > max_lanes) {
        return BT::NodeStatus::FAILURE;
    }
    const int lane_count = std::max(1, static_cast<int>(lanes_needed));
    const double margin = (width - (lane_count - 1) * coverage_step) / 2.0;
    RegionWaypoints waypoints;
    waypoints.reserve(2 * lane_count);
    for (int lane = 0; lane < lane_count; ++lane) {
        const double fraction = (margin + lane * coverage_step) / width;
        const double y = origin_y + fraction * length_y;
        const std::array<double, 2> start = {{origin_x, y}};
        const std::array<double, 2> end = {{origin_x + length_x, y}};
        waypoints.push_back(lane % 2 == 0 ? start : end);
        waypoints.push_back(lane % 2 == 0 ? end : start);
    }

    setOutput("waypoints", waypoints);//config().blackboard->set()
    setOutput("waypoint_count", static_cast<int>(waypoints.size()));
    return BT::NodeStatus::SUCCESS;
}

} // namespace maxt
