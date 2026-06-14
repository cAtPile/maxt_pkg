/**
 * @brief conditions_node.cpp
 * @details 判断降落目标是否为left或right
 * @TODO: 计划取消，在qrd中实现
 */
#include <maxt_pkg/maxt_nodes/conditions_node.hpp>

namespace maxt {

CheckLandLeftNode::CheckLandLeftNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {
}

BT::PortsList CheckLandLeftNode::providedPorts() {
    return {
        BT::InputPort<std::string>("land_target")
    };
}

BT::NodeStatus CheckLandLeftNode::tick() {
    // 从黑板中直接获取land_target值
    auto blackboard = config().blackboard;
    std::string land_target;
    
    if (blackboard->get<std::string>("land_target", land_target)) {
        // 检查land_target是否为left
        if (land_target == "left") {
            ROS_INFO("CheckLandLeftNode: land_target is left, returning SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
    } else {
        ROS_WARN("CheckLandLeftNode: Failed to get land_target from blackboard");
    }
    return BT::NodeStatus::FAILURE;
}

CheckLandRightNode::CheckLandRightNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {
}

BT::PortsList CheckLandRightNode::providedPorts() {
    return {
        BT::InputPort<std::string>("land_target")
    };
}

BT::NodeStatus CheckLandRightNode::tick() {
    // 从黑板中直接获取land_target值
    auto blackboard = config().blackboard;
    std::string land_target;
    
    if (blackboard->get<std::string>("land_target", land_target)) {
        // 检查land_target是否为right
        if (land_target == "right") {
            ROS_INFO("CheckLandRightNode: land_target is right, returning SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
    } else {
        ROS_WARN("CheckLandRightNode: Failed to get land_target from blackboard");
    }
    return BT::NodeStatus::FAILURE;
}

} // namespace maxt
