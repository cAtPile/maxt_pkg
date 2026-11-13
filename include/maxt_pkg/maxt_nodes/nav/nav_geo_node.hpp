#ifndef MAXT_NODES_NAV_GEO_HPP
#define MAXT_NODES_NAV_GEO_HPP

#include <maxt_pkg/maxt_nodes.hpp>
#include <maxt_pkg/global_nav_curve.hpp>
#include <maxt_pkg/pid_controller.hpp>

namespace maxt {

// 经纬度绝对目标导航；保持启动时的高度，不接收高度目标。
class NavGeo : public MavActionNode {
public:
    NavGeo(const std::string& name, const BT::NodeConfiguration& config,
                   MavKit& mav, const ros::NodeHandle& nh)
        : MavActionNode(name, config, mav), nh_(nh) {}

    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void applyYaw(mavros_msgs::GlobalPositionTarget& target, double elapsed, double dt);
    ros::NodeHandle nh_;
    GlobalNavCurve curve_;
    PIDController pid_east_, pid_north_;
    PIDController pid_yaw_;
    double yaw_coefficients_[6]{};
    double target_yaw_{0.0}, yaw_duration_{0.0}, yaw_tolerance_{0.0};
    double last_yaw_{0.0}, measured_yaw_{0.0};
    mavros_msgs::GlobalPositionTarget target_;
    double max_duration_{60.0};
    ros::Time start_time_, last_time_, hold_start_;
    bool holding_{false};
    double hold_timeout_{3.0};
    double altitude_raw_{0.0};
    double timeout_{60.0};
};

} // namespace maxt

#endif
