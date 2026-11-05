#ifndef MAXT_AZIMUTH_FIX_NAV_NODE_HPP
#define MAXT_AZIMUTH_FIX_NAV_NODE_HPP

#include <maxt_pkg/maxt_nodes.hpp>
#include <maxt_pkg/pid_controller.hpp>
#include <maxt_pkg/quintic_curve.hpp>

namespace maxt {

class AzimuthFixNavNode : public MavActionNode {
public:
    AzimuthFixNavNode(const std::string& name, const BT::NodeConfiguration& config,
                      MavKit& mav, const ros::NodeHandle& nh);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void publishCurve(double t, const ros::Time& now);

    ros::NodeHandle nh_;
    QuinticCurve curve_;
    PIDController pid_x_, pid_y_, pid_z_;
    double tx_{0.0}, ty_{0.0}, tz_{0.0};
    double duration_{1.0}, tolerance_{0.1}, timeout_{60.0};
    ros::Time start_time_, last_time_, hold_start_;
    bool holding_{false};
};

} // namespace maxt
#endif
