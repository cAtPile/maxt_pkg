#include <maxt_pkg/maxt_nodes/connect_check_node.hpp>

namespace maxt {

BT::PortsList ConnectCheckNode::providedPorts() {
    return {
        BT::InputPort<double>("timeout", 60.0, "connection timeout (s)"),
        BT::InputPort<double>("map_x",  0.0,  "reference map x (world)"),
        BT::InputPort<double>("map_y",  0.0,  "reference map y (world)"),
        BT::InputPort<double>("fact_x", 0.0,  "reference SLAM x"),
        BT::InputPort<double>("fact_y", 0.0,  "reference SLAM y")
    };
}

ConnectCheckNode::ConnectCheckNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav) {
}

BT::NodeStatus ConnectCheckNode::onStart() {
    if (!getInput<double>("timeout", timeout_)) {
        timeout_ = 60.0;
    }
    getInput<double>("map_x", map_x_);
    getInput<double>("map_y", map_y_);
    getInput<double>("fact_x", fact_x_);
    getInput<double>("fact_y", fact_y_);

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
        if (map_x_ != 0.0 || map_y_ != 0.0) {
            mav_.mavCalibrate(map_x_, map_y_, fact_x_, fact_y_);
        }
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
