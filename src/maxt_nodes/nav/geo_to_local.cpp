#include <maxt_pkg/maxt_nodes/nav/geo_to_local.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <cmath>

namespace maxt {

BT::PortsList Geo2Local::providedPorts() {
    return {BT::InputPort<double>("longitude"), BT::InputPort<double>("latitude"),
            BT::OutputPort<double>("x"), BT::OutputPort<double>("y")};
}

BT::NodeStatus Geo2Local::tick() {
    double longitude, latitude;
    if (!getInput("longitude", longitude) || !getInput("latitude", latitude) ||
        !std::isfinite(longitude) || std::abs(longitude) > 180 ||
        !std::isfinite(latitude) || std::abs(latitude) > 90) {
        ROS_ERROR("[Geo2Local] Invalid target longitude or latitude.");
        return BT::NodeStatus::FAILURE;
    }
    if (!mav_.hasGlobalPose() || !mav_.hasFreshNavigationData(true)) {
        ROS_ERROR("[Geo2Local] Current navigation data unavailable.");
        return BT::NodeStatus::FAILURE;
    }
    const auto global = mav_.GetGlobalPose();
    const auto local = mav_.GetRawPose().pose.position;
    // 当前定位点作为切平面原点，将东西、南北位移叠加到 MAVROS local 坐标。
    GeographicLib::LocalCartesian projection(global.latitude, global.longitude, 0.0);
    double east, north, up;
    projection.Forward(latitude, longitude, 0.0, east, north, up);
    if (!setOutput("x", local.x + east) || !setOutput("y", local.y + north)) {
        ROS_ERROR("[Geo2Local] Missing local coordinate output mapping.");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

} // namespace maxt
