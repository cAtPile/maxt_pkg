#ifndef MAXT_TARGET_CHECK_NODE_HPP
#define MAXT_TARGET_CHECK_NODE_HPP

#include <maxt_pkg/maxt_nodes.hpp>
#include <ros/ros.h>
#include <map>
#include <std_msgs/String.h>
#include <mutex>
#include <vector>

namespace maxt {

// 持续等待有效目标，并按 ID 保存最近一次记录时的原始本地位姿。
class TargetCheckNode : public MavActionNode {
public:
    TargetCheckNode(const std::string& name, const BT::NodeConfiguration& config,
                    ros::NodeHandle& nh, MavKit& mav, BT::Blackboard::Ptr history = {});

    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void onDetection(const std_msgs::String::ConstPtr& message);

    std::mutex mutex_;
    geometry_msgs::PoseStamped detected_pose_;
    std::string content_;
    BT::Blackboard::Ptr history_;
    ros::WallTime received_at_;
    // 最后声明，确保订阅在回调所访问的成员销毁前先释放。
    ros::Subscriber subscriber_;
};

} // namespace maxt

#endif
