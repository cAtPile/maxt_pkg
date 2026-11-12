#include <maxt_pkg/maxt_nodes/list_recorded_targets_node.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <algorithm>
#include <cmath>
#include <utility>
#include <limits>
#include <map>
#include <vector>

namespace maxt {

ListRecordedTargetsNode::ListRecordedTargetsNode(
    const std::string& name, const BT::NodeConfiguration& config, BT::Blackboard::Ptr history,
    std::function<geometry_msgs::PoseStamped()> current_pose)
    : BT::SyncActionNode(name, config), history_(history ? history : config.blackboard),
      current_pose_(std::move(current_pose)) {}

BT::PortsList ListRecordedTargetsNode::providedPorts() {
    return {BT::InputPort<bool>("sort_by_distance", true,
                                      "Sort by 3D distance from current raw local pose; false sorts by ID"),
            BT::OutputPort<std::vector<std::string>>("target_ids"),
            BT::OutputPort<int>("target_count")};
}

BT::NodeStatus ListRecordedTargetsNode::tick() {
    std::map<std::string, geometry_msgs::PoseStamped> poses;
    std::vector<std::string> processed, ids;
    history_->get("target_local_poses", poses);
    history_->get("processed_qr", processed);
    for (const auto& entry : poses) {
        if (std::find(processed.begin(), processed.end(), entry.first) == processed.end()) {
            ids.push_back(entry.first);
        }
    }
    if (ids.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        return BT::NodeStatus::FAILURE;
    }
    if (getInput<bool>("sort_by_distance").value() && !ids.empty()) {
        if (!current_pose_) {
            return BT::NodeStatus::FAILURE;
        }
        const auto origin = current_pose_();
        const auto valid = [](const geometry_msgs::PoseStamped& pose) {
            const auto& p = pose.pose.position;
            return !pose.header.stamp.isZero() && std::isfinite(p.x) &&
                   std::isfinite(p.y) && std::isfinite(p.z);
        };
        if (!valid(origin)) {
            return BT::NodeStatus::FAILURE;
        }
        std::map<std::string, double> distances;
        for (const auto& id : ids) {
            const auto& pose = poses.at(id);
            if (!valid(pose) || pose.header.frame_id != origin.header.frame_id) {
                return BT::NodeStatus::FAILURE;
            }
            const auto& p = pose.pose.position;
            const auto& o = origin.pose.position;
            const double distance = std::hypot(p.x - o.x, p.y - o.y, p.z - o.z);
            if (!std::isfinite(distance)) {
                return BT::NodeStatus::FAILURE;
            }
            distances.emplace(id, distance);
        }
        // ids 原本按 ID 排列，稳定排序保证等距时顺序确定。
        std::stable_sort(ids.begin(), ids.end(), [&](const std::string& a, const std::string& b) {
            return distances.at(a) < distances.at(b);
        });
    }
    setOutput("target_ids", ids);
    setOutput("target_count", static_cast<int>(ids.size()));
    return BT::NodeStatus::SUCCESS;
}

} // namespace maxt
