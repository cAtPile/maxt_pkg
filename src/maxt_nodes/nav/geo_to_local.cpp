#include <maxt_pkg/maxt_nodes/nav/geo_to_local.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <cmath>

namespace maxt {

bool geoToLocalXY(MavKit& mav, double longitude, double latitude, double& x, double& y) {
    if (!mav.hasGlobalPose() || !mav.hasFreshNavigationData(true)) {
        ROS_ERROR("[Geo2Local] Current navigation data unavailable.");
        return false;
    }
    const auto global = mav.GetGlobalPose();
    const auto local = mav.GetRawPose().pose.position;
    // 当前定位点作为切平面原点，将东西、南北位移叠加到 MAVROS local 坐标。
    GeographicLib::LocalCartesian projection(global.latitude, global.longitude, 0.0);
    double east, north, up;
    projection.Forward(latitude, longitude, 0.0, east, north, up);
    x = local.x + east;
    y = local.y + north;
    return true;
}

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
    double x, y;
    if (!geoToLocalXY(mav_, longitude, latitude, x, y)) {
        return BT::NodeStatus::FAILURE;
    }
    if (!setOutput("x", x) || !setOutput("y", y)) {
        ROS_ERROR("[Geo2Local] Missing local coordinate output mapping.");
        return BT::NodeStatus::FAILURE;
    }
    ROS_INFO("[Geo2Local] lon=%.8f deg lat=%.8f deg -> local x=%.3f m y=%.3f m",
             longitude, latitude, x, y);
    return BT::NodeStatus::SUCCESS;
}

} // namespace maxt
