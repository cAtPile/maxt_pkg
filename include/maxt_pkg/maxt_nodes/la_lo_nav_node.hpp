#ifndef MAXT_NODES_LA_LO_NAV_HPP
#define MAXT_NODES_LA_LO_NAV_HPP

#include <maxt_pkg/maxt_nodes.hpp>
#include <maxt_pkg/global_nav_curve.hpp>
#include <maxt_pkg/pid_controller.hpp>

namespace maxt {

// 经纬度绝对目标导航；保持启动时的高度，不接收高度目标。
class LaLoNavNode : public MavActionNode {
public:
    LaLoNavNode(const std::string& name, const BT::NodeConfiguration& config,
                   MavKit& mav, const ros::NodeHandle& nh)
        : MavActionNode(name, config, mav), nh_(nh) {}

    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    ros::NodeHandle nh_;
    GlobalNavCurve curve_;
    PIDController pid_east_, pid_north_;
    mavros_msgs::GlobalPositionTarget target_;
    geometry_msgs::PoseStamped start_local_pose_;
    double safety_xy_limit_{10.0};
    ros::Time start_time_, last_time_, hold_start_;
    bool holding_{false};
    double hold_timeout_{3.0};
    double altitude_raw_{0.0};
    double timeout_{60.0};
};

} // namespace maxt

#endif
