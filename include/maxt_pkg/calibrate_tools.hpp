#ifndef MAXT_CALIBRATE_TOOLS_HPP
#define MAXT_CALIBRATE_TOOLS_HPP

#include <cmath>
#include <ostream>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace maxt {

class CalibrateTools {
public:
    void setMap(double x, double y)  { map_x_ = x; map_y_ = y; recalc(); }
    void setFact(double x, double y) { fact_x_ = x; fact_y_ = y; recalc(); }

    double offset() const { return yaw_offset_; }

    // ====== 正向: 世界 -> SLAM ======

    void transRaw(double x, double y, double& ox, double& oy) const {
        ox = x * c_ - y * s_;
        oy = x * s_ + y * c_;
    }

    void transRaw(mavros_msgs::PositionTarget& t) const {
        double px = t.position.x, py = t.position.y;
        t.position.x = px * c_ - py * s_;
        t.position.y = px * s_ + py * c_;

        double vx = t.velocity.x, vy = t.velocity.y;
        t.velocity.x = vx * c_ - vy * s_;
        t.velocity.y = vx * s_ + vy * c_;

        double ax = t.acceleration_or_force.x, ay = t.acceleration_or_force.y;
        t.acceleration_or_force.x = ax * c_ - ay * s_;
        t.acceleration_or_force.y = ax * s_ + ay * c_;
    }

    void transPose(geometry_msgs::PoseStamped& p) const {
        double px = p.pose.position.x, py = p.pose.position.y;
        p.pose.position.x = px * c_ - py * s_;
        p.pose.position.y = px * s_ + py * c_;
    }

    // ====== 逆向: SLAM -> 世界 ======

    void untransRaw(double x, double y, double& ox, double& oy) const {
        ox =  x * c_ + y * s_;
        oy = -x * s_ + y * c_;
    }

    void untransRaw(mavros_msgs::PositionTarget& t) const {
        double px = t.position.x, py = t.position.y;
        t.position.x =  px * c_ + py * s_;
        t.position.y = -px * s_ + py * c_;

        double vx = t.velocity.x, vy = t.velocity.y;
        t.velocity.x =  vx * c_ + vy * s_;
        t.velocity.y = -vx * s_ + vy * c_;
    }

    void untransPose(geometry_msgs::PoseStamped& p) const {
        double px = p.pose.position.x, py = p.pose.position.y;
        p.pose.position.x =  px * c_ + py * s_;
        p.pose.position.y = -px * s_ + py * c_;
    }

    void untransTwist(geometry_msgs::TwistStamped& t) const {
        double vx = t.twist.linear.x, vy = t.twist.linear.y;
        t.twist.linear.x =  vx * c_ + vy * s_;
        t.twist.linear.y = -vx * s_ + vy * c_;
    }

    void print(std::ostream& os = std::cout) const {
        os << "map: (" << map_x_ << ", " << map_y_ << ") "
           << "fact: (" << fact_x_ << ", " << fact_y_ << ") "
           << "offset: " << yaw_offset_ << " rad ("
           << yaw_offset_ * 180.0 / M_PI << " deg)";
    }

private:
    void recalc() {
        yaw_offset_ = std::atan2(fact_y_, fact_x_) - std::atan2(map_y_, map_x_);
        c_ = std::cos(yaw_offset_);
        s_ = std::sin(yaw_offset_);
    }

    double map_x_{0}, map_y_{0};
    double fact_x_{0}, fact_y_{0};
    double yaw_offset_{0};
    double c_{1}, s_{0};
};

} // namespace maxt
#endif
