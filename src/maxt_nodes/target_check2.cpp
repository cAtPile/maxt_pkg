#include <maxt_pkg/maxt_nodes/target_check2_node.hpp>
#include <boost/algorithm/string.hpp>

namespace maxt {

TargetCheck2Node::TargetCheck2Node(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {
}

BT::PortsList TargetCheck2Node::providedPorts() {
    return {
        BT::OutputPort<bool>("target_found")
    };
}

BT::NodeStatus TargetCheck2Node::tick() {
    auto blackboard = config().blackboard;

    std::vector<std::string> current_object;
    if (!blackboard->get<std::vector<std::string>>("current_object", current_object)) {
        ROS_WARN("TargetCheck2: Failed to read current_object from blackboard");
        setOutput("target_found", false);
        return BT::NodeStatus::FAILURE;
    }

    // trim 并检查是否为 NONE
    bool is_none = false;
    for (auto& obj : current_object) {
        boost::trim(obj);
        if (obj == "NONE") {
            is_none = true;
            ROS_INFO("TargetCheck2: current_object contains NONE");
        }
    }

    if (is_none) {
        setOutput("target_found", false);
        ROS_INFO("TargetCheck2: current_object is NONE, returning FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    std::string target_1, target_2;
    blackboard->get<std::string>("target_1", target_1);
    blackboard->get<std::string>("target_2", target_2);
    boost::trim(target_1);
    boost::trim(target_2);

    bool target_found = false;
    for (const auto& obj : current_object) {
        if (!obj.empty() && (obj == target_1 || obj == target_2)) {
            target_found = true;
            ROS_INFO("TargetCheck2: [%s] matches target", obj.c_str());
        }
    }

    // 清理黑板
    current_object.clear();
    current_object.push_back("NONE");
    blackboard->set("current_object", current_object);

    setOutput("target_found", target_found);

    if (target_found) {
        ROS_INFO("TargetCheck2: Target found, returning SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }

    ROS_INFO("TargetCheck2: Target not found, returning FAILURE");
    return BT::NodeStatus::FAILURE;
}

} // namespace maxt
