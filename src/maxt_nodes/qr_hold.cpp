#include <maxt_pkg/maxt_nodes/qr_hold_node.hpp>
namespace maxt {
BT::NodeStatus QrHold::onStart() {
    initialized_ = recover_ = false;
    held_since_ = ros::SteadyTime();
    getInput("recover", recover_);
    ROS_WARN("%s: fixed-position waiting; no automatic drop/landing/mode request.", name().c_str());
    return onRunning();
}
BT::NodeStatus QrHold::onRunning() {
    if (!initialized_) {
        if (!mav_.hasFreshNavigationData()) {
            // Preflight failure must not send a fabricated zero-position target.
            recover_ = false;
            mav_.setSetpointMode(SetpointMode::STANDBY);
            return BT::NodeStatus::RUNNING;
        }
        hold_ = qrHoldTarget(mav_.GetRawPose(), mav_.get_current_yaw());
        initialized_ = true;
        held_since_ = ros::SteadyTime::now();
    }
    publishQrHold(mav_, hold_);
    if (recover_ && (!mav_.isArmed() || mav_.getMode() != "OFFBOARD" ||
                     !mav_.hasFreshNavigationData(true))) {
        recover_ = false;
    }
    if (recover_ && (ros::SteadyTime::now() - held_since_).toSec() >= 1.0)
        return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::RUNNING;
}
} // namespace maxt
