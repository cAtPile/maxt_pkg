#include <maxt_pkg/maxt_nodes/traverse_path_generator_node.hpp>
#include <maxt_pkg/survey_geometry.hpp>
#include <cmath>
#include <algorithm>

namespace maxt {

BT::PortsList TraversePathGeneratorNode::providedPorts() {
    return {
        BT::InputPort<double>("origin_x"),
        BT::InputPort<double>("origin_y"),
        BT::InputPort<double>("lenth_n"),
        BT::InputPort<double>("lenth_e"),
        BT::InputPort<double>("heading_yaw", 0.0, "Compass heading in degrees"),
        BT::InputPort<double>("coverage_step"),
        BT::OutputPort<RegionWaypoints>("waypoints", "Ordered XY waypoint array"),
        BT::OutputPort<int>("waypoint_count", "Number of waypoints")
    };
}

BT::NodeStatus TraversePathGeneratorNode::tick() {
    double origin_x, origin_y, north_length, east_length, coverage_step, heading;
    if (!getInput("origin_x", origin_x) || !getInput("origin_y", origin_y) ||
        !getInput("lenth_n", north_length) || !getInput("lenth_e", east_length) ||
        !getInput("heading_yaw", heading) || !std::isfinite(heading) ||
        !getInput("coverage_step", coverage_step) ||
        !std::isfinite(coverage_step) || coverage_step <= 0.0 ||
        north_length == 0.0 || east_length == 0.0 ||
        !std::isfinite(origin_x) || !std::isfinite(origin_y) ||
        !std::isfinite(north_length) || !std::isfinite(east_length)) {
        return BT::NodeStatus::FAILURE;
    }

    const double enu_yaw = (90.0 - heading) * M_PI / 180.0;
    // 航带沿航向往返，沿航向右侧的有符号边长推进。
    const double width = std::abs(east_length);
    // X 两端各内缩半个航带间距；短边最多缩至中点，避免端点交叉。
    const double x_margin = std::min(coverage_step / 2.0, std::abs(north_length) / 2.0);
    const double x_offset = std::copysign(x_margin, north_length);
    const double start_x = x_offset;
    const double end_x = north_length - x_offset;

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
        const double y = fraction * east_length;
        for (int end = 0; end < 2; ++end) {
            const double x = (lane + end) % 2 == 0 ? start_x : end_x;
            const auto offset = surveyOffsetToLocal(x, -y, enu_yaw);
            const double local_x = origin_x + offset[0];
            const double local_y = origin_y + offset[1];
            if (!std::isfinite(local_x) || !std::isfinite(local_y)) {
                return BT::NodeStatus::FAILURE;
            }
            waypoints.push_back({{local_x, local_y}});
        }
    }

    setOutput("waypoints", waypoints);
    setOutput("waypoint_count", static_cast<int>(waypoints.size()));
    return BT::NodeStatus::SUCCESS;
}

} // namespace maxt
