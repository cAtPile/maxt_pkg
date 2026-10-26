#include <maxt_pkg/maxt_nodes/found_check_node.hpp>

namespace maxt {

FoundCheckNode::FoundCheckNode(const std::string& name,
                               const BT::NodeConfiguration& config,
                               ros::NodeHandle& nh)
    : BT::ConditionNode(name, config) {
    const auto topic = getInput<std::string>("topic").value();
    subscriber_ = nh.subscribe(topic, 1, &FoundCheckNode::onDetection, this);
}

BT::PortsList FoundCheckNode::providedPorts() {
    return {
        BT::InputPort<std::string>("topic", "/aruco_center/is_found",
                                   "std_msgs/Bool detection topic; fixed at construction")
    };
}

void FoundCheckNode::onDetection(const std_msgs::Bool::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    found_ = message->data;
    received_at_ = ros::WallTime::now();
}

BT::NodeStatus FoundCheckNode::tick() {
    // 检测有效期每次 tick 读取，支持运行中调整。
    double timeout;
    ros::NodeHandle("~found_check").param("timeout", timeout, 0.5);
    std::lock_guard<std::mutex> lock(mutex_);
    // Bool 无时间戳，使用本地接收时间；未收到、false 或断流均视为未找到。
    const bool fresh = !received_at_.isZero() &&
        (ros::WallTime::now() - received_at_).toSec() <= timeout;
    return found_ && fresh ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace maxt
