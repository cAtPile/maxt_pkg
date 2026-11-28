#include <maxt_pkg/maxt_nodes/nav/heading_to_local.hpp>
#include <maxt_pkg/maxt_nodes/nav/geo_to_local.hpp>
#include <maxt_pkg/survey_geometry.hpp>

namespace maxt {

BT::PortsList Heading2Local::providedPorts() {
    return {BT::InputPort<double>("base_Longitude"), BT::InputPort<double>("base_Latitude"),
            BT::InputPort<double>("headingYaw"), BT::InputPort<double>("target_n"),
            BT::InputPort<double>("target_e"), BT::OutputPort<double>("x"),
            BT::OutputPort<double>("y")};
}

BT::NodeStatus Heading2Local::tick() {
    double longitude, latitude, heading_deg, forward, right;
    if (!getInput("base_Longitude", longitude) || !getInput("base_Latitude", latitude) ||
        !getInput("headingYaw", heading_deg) || !getInput("target_n", forward) ||
        !getInput("target_e", right)) {
        ROS_ERROR("[Heading2Local] Missing heading, displacement or base coordinates.");
        return BT::NodeStatus::FAILURE;
    }
    double x, y;
    if (!geoToLocalXY(mav_, longitude, latitude, x, y)) {
        return BT::NodeStatus::FAILURE;
    }
    // 罗盘航向从北顺时针计角；ENU 旋转函数使用从东逆时针计角和向左的横轴。
    const double enu_yaw = (90.0 - heading_deg) * M_PI / 180.0;
    const auto offset = surveyOffsetToLocal(forward, -right, enu_yaw);
    if (!setOutput("x", x + offset[0]) || !setOutput("y", y + offset[1])) {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

} // namespace maxt
