#pragma once
#include <maxt_pkg/maxt_nodes/region_waypoints.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace maxt {
// (0,0) and the signed opposite corner define the rectangle in takeoff FLU axes.
inline RegionWaypoints stopSurveyGrid(double corner_x, double corner_y, double step) {
    if (!std::isfinite(corner_x) || !std::isfinite(corner_y) || !std::isfinite(step) ||
        corner_x == 0 || corner_y == 0 || step <= 0)
        throw std::invalid_argument("Survey corners must be finite and nonzero; step must be positive");
    const double ax = std::abs(corner_x), ay = std::abs(corner_y);
    const double nx = std::ceil(ax/step), ny = std::ceil(ay/step);
    if (!std::isfinite(nx) || !std::isfinite(ny) || (nx+1)*(ny+1) > 10000)
        throw std::invalid_argument("Survey exceeds 10000 stops");
    RegionWaypoints points;
    for (int x = 0; x <= static_cast<int>(nx); ++x) {
        for (int j = 0; j <= static_cast<int>(ny); ++j) {
            const int y = x%2 == 0 ? j : static_cast<int>(ny)-j;
            points.push_back({{std::copysign(std::min(x*step, ax),corner_x),
                               std::copysign(std::min(y*step, ay),corner_y)}});
        }
    }
    return points;
}

// X forward, Y left; yaw is the frozen takeoff yaw in raw MAVROS local ENU.
inline std::array<double,2> surveyOffsetToLocal(double x, double y, double yaw) {
    return {{x*std::cos(yaw)-y*std::sin(yaw), x*std::sin(yaw)+y*std::cos(yaw)}};
}
} // namespace maxt
