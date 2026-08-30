#include <maxt_pkg/maxt_nodes/connect_check_node.hpp>

namespace maxt {

BT::PortsList ConnectCheckNode::providedPorts() {
    return {
        BT::InputPort<double>("timeout", 60.0, "connection timeout (s)")
    };
}

ConnectCheckNode::ConnectCheckNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav) {
}

BT::NodeStatus ConnectCheckNode::onStart() {
    if (!getInput<double>("timeout", timeout_)) {
        timeout_ = 60.0;
    }
    start_time_ = ros::WallTime::now();
    ROS_INFO("ConnectCheckNode: Waiting for drone connection...");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ConnectCheckNode::onRunning() {
    if ((ros::WallTime::now() - start_time_).toSec() > timeout_) {
        ROS_ERROR("ConnectCheckNode: Connection timeout!");
        return BT::NodeStatus::FAILURE;
    }

    if (mav_.isConnected()) {
        ROS_INFO("ConnectCheckNode: Drone connected.");
        return BT::NodeStatus::SUCCESS;
    }

    ROS_INFO_THROTTLE(1.0, "ConnectCheckNode: Waiting for drone connection...");
    return BT::NodeStatus::RUNNING;
}

void ConnectCheckNode::onHalted() {
    ROS_WARN("ConnectCheckNode: Halted during connection check!");
}

} // namespace maxt
