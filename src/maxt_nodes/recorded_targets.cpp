#include <maxt_pkg/maxt_nodes/recorded_targets_node.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <map>
#include <vector>

namespace maxt {

GetRecordedTargetNode::GetRecordedTargetNode(
    const std::string& name, const BT::NodeConfiguration& config, BT::Blackboard::Ptr history)
    : BT::SyncActionNode(name, config), history_(history ? history : config.blackboard) {}

BT::PortsList GetRecordedTargetNode::providedPorts() {
    return {BT::InputPort<std::vector<std::string>>("target_ids"),
            BT::InputPort<int>("index"), BT::OutputPort<int>("next_index"),
            BT::OutputPort<std::string>("target_id"),
            BT::OutputPort<double>("x"), BT::OutputPort<double>("y"),
            BT::OutputPort<double>("z")};
}

BT::NodeStatus GetRecordedTargetNode::tick() {
    std::vector<std::string> ids;
    int index;
    if (!getInput("target_ids", ids) || !getInput("index", index) ||
        index < 0 || static_cast<std::size_t>(index) >= ids.size()) {
        return BT::NodeStatus::FAILURE;
    }
    std::map<std::string, geometry_msgs::PoseStamped> poses;
    if (!history_->get("target_local_poses", poses)) return BT::NodeStatus::FAILURE;
    const auto it = poses.find(ids[index]);
    if (it == poses.end() || it->second.header.stamp.isZero()) return BT::NodeStatus::FAILURE;
    const auto& p = it->second.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        return BT::NodeStatus::FAILURE;
    }
    setOutput("target_id", it->first);
    setOutput("x", p.x);
    setOutput("y", p.y);
    setOutput("z", p.z);
    setOutput("next_index", index + 1);
    return BT::NodeStatus::SUCCESS;
}

} // namespace maxt
