#pragma once
#include <maxt_pkg/maxt_nodes.hpp>
#include <maxt_pkg/qr_survey_support.hpp>
namespace maxt {
// Deliberately never returns SUCCESS: MaxtCore must keep its timers/process alive.
class QrHold : public MavActionNode {
    mavros_msgs::PositionTarget hold_;
    ros::SteadyTime held_since_;
    bool initialized_{false}, recover_{false};
public:
    QrHold(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
        : MavActionNode(name, config, mav) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<bool>("recover", false, "Permit navigation after a short fixed hold")};
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {
        mav_.setSetpointMode(initialized_ ? SetpointMode::SLOW_DOWN : SetpointMode::STANDBY);
    }
};
} // namespace maxt
