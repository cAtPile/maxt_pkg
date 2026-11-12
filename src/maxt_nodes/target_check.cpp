#include <maxt_pkg/maxt_nodes/target_check_node.hpp>
#include <algorithm>

namespace maxt {

TargetCheckNode::TargetCheckNode(const std::string& name,
                               const BT::NodeConfiguration& config,
                               ros::NodeHandle& nh, MavKit& mav, BT::Blackboard::Ptr history)
    : MavActionNode(name, config, mav),
      history_(history ? history : config.blackboard) {
    std::vector<std::string> processed;
    if (!history_->get("processed_qr", processed)) {
        history_->set("processed_qr", processed);
    }
    std::map<std::string, geometry_msgs::PoseStamped> poses;
    if (!history_->get("target_local_poses", poses)) {
        history_->set("target_local_poses", poses);
    }
    const auto topic = getInput<std::string>("topic").value();
    subscriber_ = nh.subscribe(topic, 1, &TargetCheckNode::onDetection, this);
}

BT::PortsList TargetCheckNode::providedPorts() {
    return {
        BT::InputPort<std::string>("topic", "/aruco_center/content",
                                   "std_msgs/String content; empty means no target"),
        BT::InputPort<bool>("record", true,
                           "true: reject processed IDs and save local pose; false: visibility check only"),
        BT::OutputPort<std::string>("target_id", "ID captured by a successful detection"),
        BT::OutputPort<geometry_msgs::PoseStamped>(
            "target_pose", "Raw MAVROS local pose when the target was detected")
    };
}

void TargetCheckNode::onDetection(const std_msgs::String::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    content_ = message->data;
    detected_pose_ = mav_.GetRawPose();
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
    if (getInput<bool>("record").value()) {
        // 尚未收到定位时，不把默认零位姿登记为目标观察点。
        if (detected_pose_.header.stamp.isZero()) {
            return BT::NodeStatus::RUNNING;
        }
        auto poses = history_->get<std::map<std::string, geometry_msgs::PoseStamped>>(
            "target_local_poses");
        // 更新该目标的最新位姿；发现记录与 processed_qr 的完成记录独立。
        poses[content_] = detected_pose_;
        history_->set("target_local_poses", poses);
    }
    setOutput("target_pose", detected_pose_);
    setOutput("target_id", content_);
    return BT::NodeStatus::SUCCESS;
}

void TargetCheckNode::onHalted() {
    // 没有后台任务；保留检测快照，重启时仍按消息有效期检查。
}

} // namespace maxt
