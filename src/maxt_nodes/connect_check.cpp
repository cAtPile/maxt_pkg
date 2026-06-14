/**
 * @brief connect_check_node.cpp
 * @details 检查无人机连接状态
 * @TODO: 
 */
#include <maxt_pkg/maxt_nodes/connect_check_node.hpp>

namespace maxt {

BT::PortsList ConnectCheckNode::providedPorts() {
    return {
        BT::InputPort<double>("timeout", 60.0, "连接超时时间(s)")
    };
}

ConnectCheckNode::ConnectCheckNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav) {
}

BT::NodeStatus ConnectCheckNode::onStart() {
    // 1. 获取超时参数
    if (!getInput<double>("timeout", timeout_)) {
        timeout_ = 60.0;
    }

    // 2. 记录开始时间
    start_time_ = ros::WallTime::now();
    ROS_INFO("ConnectCheckNode: Waiting for drone connection...");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ConnectCheckNode::onRunning() {
    // A. 超时检查
    if ((ros::WallTime::now() - start_time_).toSec() > timeout_) {
        ROS_ERROR("ConnectCheckNode: Connection timeout!");
        return BT::NodeStatus::FAILURE;
    }

    // B. 检查连接状态
    if (mav_.isConnected()) {
        ROS_INFO("ConnectCheckNode: Drone connected, initialized delivery state.");
        return BT::NodeStatus::SUCCESS;
    }

    // C. 还在等待连接
    ROS_INFO_THROTTLE(1.0, "ConnectCheckNode: Waiting for drone connection...");
    return BT::NodeStatus::RUNNING;
}

void ConnectCheckNode::onHalted() {
    //TODO：是否需要处理halt事件
    ROS_WARN("ConnectCheckNode: Halted during connection check!");
}

} // namespace maxt