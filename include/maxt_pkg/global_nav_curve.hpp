#ifndef MAXT_GLOBAL_NAV_CURVE_HPP
#define MAXT_GLOBAL_NAV_CURVE_HPP

#include <maxt_pkg/quintic_curve.hpp>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <GeographicLib/Geodesic.hpp>
#include <algorithm>
#include <cmath>

namespace maxt {

// 水平使用 WGS84 测地线。可选相对高度曲线，旧调用仍保持起点高度。
class GlobalNavCurve {
public:
    void setTarget(const mavros_msgs::GlobalPositionTarget& start,
                   const mavros_msgs::GlobalPositionTarget& target,
                   double v_max, double a_max, bool change_altitude = false) {
        start_ = start;
        target_ = target;
        if (!change_altitude) target_.altitude = start.altitude;
        double distance, end_azimuth;
        GeographicLib::Geodesic::WGS84().Inverse(
            start.latitude, start.longitude, target.latitude, target.longitude,
            distance, azimuth_, end_azimuth);
        // GeographicLib on this platform can return ~1e-11 m for identical LLA.
        // A stationary command must have exactly zero horizontal derivatives.
        if (distance < 1e-8) distance = 0.0;
        const double spatial_distance = std::hypot(distance, target_.altitude-start.altitude);
        const double t_vel = spatial_distance * 1.875 / (0.9 * v_max);
        const double t_acc = std::sqrt(spatial_distance * 5.774 / a_max);
        duration_ = 1.2 * std::max({t_vel, t_acc, 0.5});
        QuinticCurve::solveAxis(0.0, 0.0, 0.0, distance, 0.0, 0.0, duration_, coefficients_);
        std::fill(std::begin(altitude_coefficients_), std::end(altitude_coefficients_), 0.0);
        altitude_coefficients_[0] = start.altitude;
        if (target_.altitude != start.altitude)
            QuinticCurve::solveAxis(start.altitude, 0.0, 0.0, target_.altitude,
                                   0.0, 0.0, duration_, altitude_coefficients_);
    }

    mavros_msgs::GlobalPositionTarget sample(double t) const {
        auto target = target_;
        target.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_YAW |
                           mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
        target.velocity = geometry_msgs::Vector3();
        target.acceleration_or_force = geometry_msgs::Vector3();
        if (t >= duration_) return target;

        const double time = std::max(0.0, t);
        const double distance = QuinticCurve::evalPos(coefficients_, time);
        const double speed = QuinticCurve::evalVel(coefficients_, time);
        const double accel = QuinticCurve::evalAcc(coefficients_, time);
        target.altitude = QuinticCurve::evalPos(altitude_coefficients_, time);
        target.velocity.z = QuinticCurve::evalVel(altitude_coefficients_, time);
        target.acceleration_or_force.z = QuinticCurve::evalAcc(altitude_coefficients_, time);
        double azimuth;
        GeographicLib::Geodesic::WGS84().Direct(
            start_.latitude, start_.longitude, azimuth_, distance,
            target.latitude, target.longitude, azimuth);
        const double heading = azimuth * M_PI / 180.0;
        // MAVROS 的全球 raw 速度/加速度输入为 ENU，由插件转换成 MAVLink NED。
        target.velocity.x = speed * std::sin(heading);
        target.velocity.y = speed * std::cos(heading);
        target.acceleration_or_force.x = accel * std::sin(heading);
        target.acceleration_or_force.y = accel * std::cos(heading);
        return target;
    }

    double duration() const { return duration_; }

private:
    mavros_msgs::GlobalPositionTarget start_, target_;
    double coefficients_[6]{};
    double altitude_coefficients_[6]{};
    double azimuth_{0.0};
    double duration_{1.0};
};

} // namespace maxt

#endif
