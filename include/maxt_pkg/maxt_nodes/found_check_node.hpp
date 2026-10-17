#ifndef MAXT_FOUND_CHECK_NODE_HPP
#define MAXT_FOUND_CHECK_NODE_HPP

#include <behaviortree_cpp_v3/condition_node.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <mutex>

namespace maxt {

// 只读取检测结果；由 ReactiveFallback 在导航期间持续检查。
class FoundCheckNode : public BT::ConditionNode {
public:
    FoundCheckNode(const std::string& name, const BT::NodeConfiguration& config,
                   ros::NodeHandle& nh);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    void onDetection(const std_msgs::Bool::ConstPtr& message);

    std::mutex mutex_;
    bool found_{false};
    ros::WallTime received_at_;
    // 最后声明，确保订阅在回调所访问的成员销毁前先释放。
    ros::Subscriber subscriber_;
};

} // namespace maxt

#endif
