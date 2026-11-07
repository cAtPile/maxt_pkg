#ifndef MAXT_GLOBAL_NAV_CURVE_HPP
#define MAXT_GLOBAL_NAV_CURVE_HPP

#include <maxt_pkg/quintic_curve.hpp>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <GeographicLib/Geodesic.hpp>
#include <algorithm>
#include <cmath>

namespace maxt {

// 沿 WGS-84 测地线规划水平距离，高度按目标指定的基准独立插值，边界速度、加速度为零。
class GlobalNavCurve {
public:
    void setTarget(const mavros_msgs::GlobalPositionTarget& start,
                   const mavros_msgs::GlobalPositionTarget& target,
                   double v_max, double a_max) {
        start_ = start;
        target_ = target;
        double distance, end_azimuth;
        GeographicLib::Geodesic::WGS84().Inverse(
            start.latitude, start.longitude, target.latitude, target.longitude,
            distance, azimuth_, end_azimuth);
        const double length = std::hypot(distance, target.altitude - start.altitude);
        const double t_vel = length * 1.875 / (0.9 * v_max);
        const double t_acc = std::sqrt(length * 5.774 / a_max);
        duration_ = 1.2 * std::max({t_vel, t_acc, 0.5});
        curve_.setWaypoints(0.0, 0.0, start.altitude,
                            distance, 0.0, target.altitude, duration_);
    }

    mavros_msgs::GlobalPositionTarget sample(double t) const {
        auto target = target_;
        target.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_YAW |
                           mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
        target.velocity = geometry_msgs::Vector3();
        target.acceleration_or_force = geometry_msgs::Vector3();
        if (t >= duration_) return target;

        double distance, unused_p, altitude, speed, unused_v, vz, accel, unused_a, az;
        curve_.evaluate(std::max(0.0, t), distance, unused_p, altitude,
                        speed, unused_v, vz, accel, unused_a, az);
        double azimuth;
        GeographicLib::Geodesic::WGS84().Direct(
            start_.latitude, start_.longitude, azimuth_, distance,
            target.latitude, target.longitude, azimuth);
        target.altitude = altitude;
        const double heading = azimuth * M_PI / 180.0;
        // MAVROS 的全球 raw 速度/加速度输入为 ENU，由插件转换成 MAVLink NED。
        target.velocity.x = speed * std::sin(heading);
        target.velocity.y = speed * std::cos(heading);
        target.velocity.z = vz;
        target.acceleration_or_force.x = accel * std::sin(heading);
        target.acceleration_or_force.y = accel * std::cos(heading);
        target.acceleration_or_force.z = az;
        return target;
    }

    double duration() const { return duration_; }

private:
    mavros_msgs::GlobalPositionTarget start_, target_;
    QuinticCurve curve_;
    double azimuth_{0.0};
    double duration_{1.0};
};

} // namespace maxt

#endif
