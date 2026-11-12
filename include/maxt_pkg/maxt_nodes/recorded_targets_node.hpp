#ifndef MAXT_RECORDED_TARGETS_NODE_HPP
#define MAXT_RECORDED_TARGETS_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>

namespace maxt {

// 读取列表中的目标 ID 和原始本地坐标，成功后推进索引。
class GetRecordedTargetNode : public BT::SyncActionNode {
public:
    GetRecordedTargetNode(const std::string& name, const BT::NodeConfiguration& config,
                          BT::Blackboard::Ptr history);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    BT::Blackboard::Ptr history_;
};

} // namespace maxt
#endif
