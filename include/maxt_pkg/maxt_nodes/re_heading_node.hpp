#ifndef MAXT_RE_HEADING_NODE_HPP
#define MAXT_RE_HEADING_NODE_HPP

#include <maxt_pkg/maxt_nodes.hpp>
#include <maxt_pkg/pid_controller.hpp>
#include <maxt_pkg/quintic_curve.hpp>

namespace maxt {

class ReHeadingNode : public MavActionNode {
public:
    ReHeadingNode(const std::string& name, const BT::NodeConfiguration& config,
                  MavKit& mav, const ros::NodeHandle& nh);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    void publish(double yaw, double yaw_rate, bool rate_control);
    ros::NodeHandle nh_;
    geometry_msgs::Point position_;
    double target_yaw_{0.0}, duration_{0.0}, timeout_{60.0};
    double tolerance_{0.0}, hold_timeout_{3.0};
    double coefficients_[6]{};
    // 连续展开测量角，避免跨越 +/-pi 时 PID 微分跳变。
    double last_yaw_{0.0}, measured_yaw_{0.0}, unwrapped_target_{0.0};
    ros::Time start_time_, last_time_, hold_start_;
    bool holding_{false};
    PIDController pid_;
};

} // namespace maxt
#endif
