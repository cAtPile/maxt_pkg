#include <maxt_pkg/maxt_nodes/target_check_node.hpp>
#include <algorithm>
#include <cmath>

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
    std::map<std::string, uint64_t> counts;
    if (!history_->get("target_local_pose_sample_counts", counts)) {
        history_->set("target_local_pose_sample_counts", counts);
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
            "target_pose", "Mean observed local position when recording; latest raw pose otherwise")
    };
}

void TargetCheckNode::onDetection(const std_msgs::String::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    content_ = message->data;
    detected_pose_ = mav_.GetRawPose();
    received_at_ = ros::WallTime::now();
    double timeout;
    ros::NodeHandle("~found_check").param("timeout", timeout, 0.5);
    while (!pending_observations_.empty() &&
           (received_at_ - pending_observations_.front().received_at).toSec() > timeout) {
        pending_observations_.pop_front();
    }
    if (!content_.empty()) {
        pending_observations_.push_back({content_, detected_pose_, received_at_});
    }
}

BT::NodeStatus TargetCheckNode::onStart() {
    return onRunning();
}

BT::NodeStatus TargetCheckNode::onRunning() {
    double timeout;
    ros::NodeHandle("~found_check").param("timeout", timeout, 0.5);
    std::lock_guard<std::mutex> lock(mutex_);
    const bool record = getInput<bool>("record").value();
    const auto processed = history_->get<std::vector<std::string>>("processed_qr");
    auto poses = history_->get<std::map<std::string, geometry_msgs::PoseStamped>>(
        "target_local_poses");
    auto counts = history_->get<std::map<std::string, uint64_t>>(
        "target_local_pose_sample_counts");
    // 每条消息只消费一次；共享累计次数，使不同航段的节点继续同一目标的平均。
    for (const auto& sample : pending_observations_) {
        const auto& p = sample.pose.pose.position;
        if (!record || sample.pose.header.stamp.isZero() ||
            (ros::WallTime::now() - sample.received_at).toSec() > timeout ||
            !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            std::find(processed.begin(), processed.end(), sample.id) != processed.end()) {
            continue;
        }
        auto& count = counts[sample.id];
        auto& mean = poses[sample.id];
        // 没有累计次数的旧记录由首个新样本重新初始化。
        if (count == 0) {
            mean = sample.pose;
        } else {
            const double weight = 1.0 / static_cast<double>(count + 1);
            mean.pose.position.x += (p.x - mean.pose.position.x) * weight;
            mean.pose.position.y += (p.y - mean.pose.position.y) * weight;
            mean.pose.position.z += (p.z - mean.pose.position.z) * weight;
            mean.header = sample.pose.header;
            mean.pose.orientation = sample.pose.pose.orientation;
        }
        ++count;
    }
    pending_observations_.clear();
    if (record) {
        history_->set("target_local_poses", poses);
        history_->set("target_local_pose_sample_counts", counts);
    }
    if (content_.empty() || received_at_.isZero() ||
        (ros::WallTime::now() - received_at_).toSec() > timeout ||
        (record && (std::find(processed.begin(), processed.end(), content_) != processed.end() ||
                    poses.find(content_) == poses.end()))) {
        return BT::NodeStatus::RUNNING;
    }
    setOutput("target_pose", record ? poses.at(content_) : detected_pose_);
    setOutput("target_id", content_);
    return BT::NodeStatus::SUCCESS;
}

void TargetCheckNode::onHalted() {
    // 没有后台任务；保留检测快照，重启时仍按消息有效期检查。
}

} // namespace maxt
