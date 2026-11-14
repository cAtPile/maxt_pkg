#ifndef MAXT_NAV_AZIMUTH_NODE_HPP
#define MAXT_NAV_AZIMUTH_NODE_HPP

#include <maxt_pkg/maxt_nodes.hpp>
#include <maxt_pkg/quintic_curve.hpp>

namespace maxt {

class NavAzimuth : public MavActionNode {
public:
    NavAzimuth(const std::string& name, const BT::NodeConfiguration& config,
                      MavKit& mav, const ros::NodeHandle& nh);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void publishCurve(double t, const ros::Time& now);

    ros::NodeHandle nh_;
    QuinticCurve curve_;
    double tx_{0.0}, ty_{0.0}, tz_{0.0};
    double duration_{1.0}, tolerance_{0.1}, timeout_{60.0};
    ros::Time start_time_, last_time_, hold_start_;
    bool holding_{false};
    double hold_timeout_{3.0};
};

} // namespace maxt
#endif
