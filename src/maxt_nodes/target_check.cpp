#include <maxt_pkg/maxt_nodes/target_check_node.hpp>
#include <algorithm>

namespace maxt {

TargetCheckNode::TargetCheckNode(const std::string& name,
                               const BT::NodeConfiguration& config,
                               ros::NodeHandle& nh, BT::Blackboard::Ptr history)
    : BT::StatefulActionNode(name, config),
      history_(history ? history : config.blackboard) {
    std::vector<std::string> processed;
    if (!history_->get("processed_qr", processed)) {
        history_->set("processed_qr", processed);
    }
    const auto topic = getInput<std::string>("topic").value();
    subscriber_ = nh.subscribe(topic, 1, &TargetCheckNode::onDetection, this);
}

BT::PortsList TargetCheckNode::providedPorts() {
    return {
        BT::InputPort<std::string>("topic", "/aruco_center/content",
                                   "std_msgs/String content; empty means no target"),
        BT::InputPort<bool>("record", true,
                           "true: reject processed IDs; false: visibility check only"),
        BT::OutputPort<std::string>("target_id", "ID captured by a successful detection")
    };
}

void TargetCheckNode::onDetection(const std_msgs::String::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    content_ = message->data;
    received_at_ = ros::WallTime::now();
}

BT::NodeStatus TargetCheckNode::onStart() {
    return onRunning();
}

BT::NodeStatus TargetCheckNode::onRunning() {
    double timeout;
    ros::NodeHandle("~found_check").param("timeout", timeout, 0.5);
    std::lock_guard<std::mutex> lock(mutex_);
    if (content_.empty() || received_at_.isZero() ||
        (ros::WallTime::now() - received_at_).toSec() > timeout) {
        return BT::NodeStatus::RUNNING;
    }
    // 仅在行为树 tick 线程访问历史；回调只更新本实例的检测快照。
    if (getInput<bool>("record").value()) {
        const auto processed = history_->get<std::vector<std::string>>("processed_qr");
        if (std::find(processed.begin(), processed.end(), content_) != processed.end()) {
            return BT::NodeStatus::RUNNING;
        }
    }
    setOutput("target_id", content_);
    return BT::NodeStatus::SUCCESS;
}

void TargetCheckNode::onHalted() {
    // 没有后台任务；保留检测快照，重启时仍按消息有效期检查。
}

} // namespace maxt
