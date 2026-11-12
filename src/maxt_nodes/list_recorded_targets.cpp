#include <maxt_pkg/maxt_nodes/list_recorded_targets_node.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <algorithm>
#include <limits>
#include <map>
#include <vector>

namespace maxt {

ListRecordedTargetsNode::ListRecordedTargetsNode(
    const std::string& name, const BT::NodeConfiguration& config, BT::Blackboard::Ptr history)
    : BT::SyncActionNode(name, config), history_(history ? history : config.blackboard) {}

BT::PortsList ListRecordedTargetsNode::providedPorts() {
    return {BT::OutputPort<std::vector<std::string>>("target_ids"),
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
    setOutput("target_ids", ids);
    setOutput("target_count", static_cast<int>(ids.size()));
    return BT::NodeStatus::SUCCESS;
}

} // namespace maxt
