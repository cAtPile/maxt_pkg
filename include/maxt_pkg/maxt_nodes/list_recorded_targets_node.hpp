#ifndef MAXT_LIST_RECORDED_TARGETS_NODE_HPP
#define MAXT_LIST_RECORDED_TARGETS_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>

namespace maxt {

// 从任务黑板列出尚未处理的目标，按 ID 排序。
class ListRecordedTargetsNode : public BT::SyncActionNode {
public:
    ListRecordedTargetsNode(const std::string& name, const BT::NodeConfiguration& config,
                            BT::Blackboard::Ptr history);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    BT::Blackboard::Ptr history_;
};

} // namespace maxt

#endif
