#ifndef MAXT_FOUND_CHECK_NODE_HPP
#define MAXT_FOUND_CHECK_NODE_HPP

#include <behaviortree_cpp_v3/condition_node.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mutex>
#include <vector>

namespace maxt {

// 按内容去重；所有实例使用任务根黑板共享 processed_qr。
class FoundCheckNode : public BT::ConditionNode {
public:
    FoundCheckNode(const std::string& name, const BT::NodeConfiguration& config,
                   ros::NodeHandle& nh, BT::Blackboard::Ptr history = {});

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    void onDetection(const std_msgs::String::ConstPtr& message);

    std::mutex mutex_;
    std::string content_;
    BT::Blackboard::Ptr history_;
    ros::WallTime received_at_;
    // 最后声明，确保订阅在回调所访问的成员销毁前先释放。
    ros::Subscriber subscriber_;
};

} // namespace maxt

#endif
