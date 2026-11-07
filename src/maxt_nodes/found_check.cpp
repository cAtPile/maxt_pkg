#include <maxt_pkg/maxt_nodes/found_check_node.hpp>
#include <algorithm>

namespace maxt {

FoundCheckNode::FoundCheckNode(const std::string& name,
                               const BT::NodeConfiguration& config,
                               ros::NodeHandle& nh, BT::Blackboard::Ptr history)
    : BT::ConditionNode(name, config),
      history_(history ? history : config.blackboard) {
    std::vector<std::string> processed;
    if (!history_->get("processed_qr", processed)) {
        history_->set("processed_qr", processed);
    }
    const auto topic = getInput<std::string>("topic").value();
    subscriber_ = nh.subscribe(topic, 1, &FoundCheckNode::onDetection, this);
}

BT::PortsList FoundCheckNode::providedPorts() {
    return {
        BT::InputPort<std::string>("topic", "/aruco_center/content",
                                   "std_msgs/String content; empty means no target"),
        BT::InputPort<bool>("record", true,
                           "false: visibility check only; do not read or update history")
    };
}

void FoundCheckNode::onDetection(const std_msgs::String::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    content_ = message->data;
    received_at_ = ros::WallTime::now();
}

BT::NodeStatus FoundCheckNode::tick() {
    double timeout;
    ros::NodeHandle("~found_check").param("timeout", timeout, 0.5);
    std::lock_guard<std::mutex> lock(mutex_);
    if (content_.empty() || received_at_.isZero() ||
        (ros::WallTime::now() - received_at_).toSec() > timeout) {
        return BT::NodeStatus::FAILURE;
    }
    if (!getInput<bool>("record").value()) {
        return BT::NodeStatus::SUCCESS;
    }
    // 仅在行为树 tick 线程访问历史；回调只更新本实例的检测快照。
    auto processed = history_->get<std::vector<std::string>>("processed_qr");
    if (std::find(processed.begin(), processed.end(), content_) != processed.end()) {
        return BT::NodeStatus::FAILURE;
    }
    processed.push_back(content_);
    history_->set("processed_qr", processed);
    return BT::NodeStatus::SUCCESS;
}

} // namespace maxt
