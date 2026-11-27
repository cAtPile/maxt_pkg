#include <maxt_pkg/maxt_nodes/polygon_coverage_wpg.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace maxt {

BT::PortsList PolygonCoverageWpG::providedPorts() {
    BT::PortsList ports = {
        BT::InputPort<double>("coverage_step"), BT::InputPort<double>("work_hight"),
        BT::OutputPort<double>("z"), BT::OutputPort<RegionWaypoints>("waypoints"),
        BT::OutputPort<int>("waypoint_count")
    };
    for (int i = 1; i <= 4; ++i) {
        ports.insert(BT::InputPort<double>("Longitude" + std::to_string(i)));
        ports.insert(BT::InputPort<double>("Latitude" + std::to_string(i)));
    }
    return ports;
}

BT::NodeStatus PolygonCoverageWpG::tick() {
    double step, height;
    if (!getInput("coverage_step", step) || !std::isfinite(step) || step <= 0 ||
        !getInput("work_hight", height) || !std::isfinite(height))
        return BT::NodeStatus::FAILURE;

    GeographicLib::LocalCartesian frame;
    RegionWaypoints corners(4);
    double cx = 0, cy = 0;
    for (int i = 0; i < 4; ++i) {
        double lon, lat, up;
        const auto suffix = std::to_string(i + 1);
        if (!getInput("Longitude" + suffix, lon) || !(std::abs(lon) <= 180) ||
            !getInput("Latitude" + suffix, lat) || !(std::abs(lat) <= 90))
            return BT::NodeStatus::FAILURE;
        if (i == 0) {
            frame.Reset(lat, lon, 0);
        }
        frame.Forward(lat, lon, 0, corners[i][0], corners[i][1], up);
        cx += corners[i][0] / 4;
        cy += corners[i][1] / 4;
    }
    // 沿质心极角排列四个角，再用连续转向验证凸性。
    std::sort(corners.begin(), corners.end(), [=](const auto& a, const auto& b) {
        return std::atan2(a[1]-cy, a[0]-cx) < std::atan2(b[1]-cy, b[0]-cx);
    });
    double ymin = corners.front()[1], ymax = ymin;
    for (int i = 0; i < 4; ++i) {
        const auto& a = corners[i];
        const auto& b = corners[(i+1)%4];
        const auto& c = corners[(i+2)%4];
        if ((b[0]-a[0])*(c[1]-b[1]) <= (b[1]-a[1])*(c[0]-b[0]))
            return BT::NodeStatus::FAILURE;
        ymin = std::min(ymin, a[1]);
        ymax = std::max(ymax, a[1]);
    }
    const double lanes = std::ceil((ymax-ymin)/step);
    if (lanes < 1 || lanes > 100000) return BT::NodeStatus::FAILURE;
    RegionWaypoints path;
    // 每条东西向扫描线位于等宽航带中心，按半开区间处理顶点交点。
    for (int i = 0; i < lanes; ++i) {
        const double y = ymin + (i + 0.5) * ((ymax-ymin)/lanes);
        double left = std::numeric_limits<double>::infinity(), right = -left;
        for (int j = 0; j < 4; ++j) {
            const auto& a = corners[j];
            const auto& b = corners[(j+1)%4];
            if ((a[1] > y) == (b[1] > y)) continue;
            const double x = a[0] + (y-a[1])/(b[1]-a[1])*(b[0]-a[0]);
            left = std::min(left, x);
            right = std::max(right, x);
        }
        if (left > right) return BT::NodeStatus::FAILURE;
        if (i % 2) std::swap(left, right);
        for (double x : {left, right}) {
            double latitude, longitude, altitude;
            frame.Reverse(x, y, 0, latitude, longitude, altitude);
            path.push_back({longitude, latitude});
        }
    }
    setOutput("waypoints", path);
    setOutput("waypoint_count", static_cast<int>(path.size()));
    setOutput("z", height);
    return BT::NodeStatus::SUCCESS;
}
} // namespace maxt
