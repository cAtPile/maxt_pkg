#include <maxt_pkg/maxt_nodes/qr_hold_node.hpp>
namespace maxt {
BT::NodeStatus QrHold::onStart() {
    initialized_ = false;
    ROS_WARN("%s: fixed-position waiting; no automatic drop/landing/mode request.", name().c_str());
    return onRunning();
}
BT::NodeStatus QrHold::onRunning() {
    if (!initialized_) {
        if (!mav_.hasFreshNavigationData()) {
            // Preflight failure must not send a fabricated zero-position target.
            mav_.setSetpointMode(SetpointMode::STANDBY);
            return BT::NodeStatus::RUNNING;
        }
        hold_ = qrHoldTarget(mav_.GetRawPose(), mav_.get_current_yaw());
        initialized_ = true;
    }
    publishQrHold(mav_, hold_);
    return BT::NodeStatus::RUNNING;
}
} // namespace maxt
