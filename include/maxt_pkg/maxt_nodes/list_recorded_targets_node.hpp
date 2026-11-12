#ifndef MAXT_LIST_RECORDED_TARGETS_NODE_HPP
#define MAXT_LIST_RECORDED_TARGETS_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/PoseStamped.h>
#include <functional>

namespace maxt {

// 从任务黑板列出尚未处理的目标，默认按距当前无人机位置的三维距离排序。
class ListRecordedTargetsNode : public BT::SyncActionNode {
public:
    ListRecordedTargetsNode(const std::string& name, const BT::NodeConfiguration& config,
                            BT::Blackboard::Ptr history,
                            std::function<geometry_msgs::PoseStamped()> current_pose = {});
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    BT::Blackboard::Ptr history_;
    std::function<geometry_msgs::PoseStamped()> current_pose_;
};

} // namespace maxt

#endif
