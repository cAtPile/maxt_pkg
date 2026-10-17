#include <maxt_pkg/maxt_nodes/region_traverse_node.hpp>
#include <cmath>
#include <algorithm>

namespace maxt {

BT::PortsList RegionTraverseNode::providedPorts() {
    return {
        BT::InputPort<double>("reg_point_1_x"),
        BT::InputPort<double>("reg_point_1_y"),
        BT::InputPort<double>("reg_point_2_x"),
        BT::InputPort<double>("reg_point_2_y"),
        BT::InputPort<double>("reg_point_3_x"),
        BT::InputPort<double>("reg_point_3_y"),
        BT::InputPort<double>("reg_point_4_x"),
        BT::InputPort<double>("reg_point_4_y"),
        BT::InputPort<double>("coverage_step"),
        BT::OutputPort<RegionWaypoints>("waypoints", "Ordered XY waypoint array"),
        BT::OutputPort<int>("waypoint_count", "Number of waypoints")
    };
}

BT::NodeStatus RegionTraverseNode::tick() {
    std::array<std::array<double, 2>, 4> corners;

    for (std::size_t i = 0; i < corners.size(); ++i) {
        const std::string prefix = "reg_point_" + std::to_string(i + 1);
        if (!getInput(prefix + "_x", corners[i][0]) ||
            !getInput(prefix + "_y", corners[i][1])) {
            return BT::NodeStatus::FAILURE;
        }
    }
    double coverage_step;
    if (!getInput("coverage_step", coverage_step) || coverage_step <= 0.0) {
        return BT::NodeStatus::FAILURE;
    }

    // 四角沿矩形边界顺序输入。航带平行于 1->2，沿 1->4 推进，支持旋转矩形。
    const double width = std::hypot(corners[3][0] - corners[0][0],
                                    corners[3][1] - corners[0][1]);

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
        const std::array<double, 2> start = {{
            corners[0][0] + fraction * (corners[3][0] - corners[0][0]),
            corners[0][1] + fraction * (corners[3][1] - corners[0][1])}};
        const std::array<double, 2> end = {{
            corners[1][0] + fraction * (corners[2][0] - corners[1][0]),
            corners[1][1] + fraction * (corners[2][1] - corners[1][1])}};
        waypoints.push_back(lane % 2 == 0 ? start : end);
        waypoints.push_back(lane % 2 == 0 ? end : start);
    }

    setOutput("waypoints", waypoints);//config().blackboard->set()
    setOutput("waypoint_count", static_cast<int>(waypoints.size()));
    return BT::NodeStatus::SUCCESS;
}

} // namespace maxt
