#ifndef MAXT_SEND_GEO_POSE_NODE_HPP
#define MAXT_SEND_GEO_POSE_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <mutex>

namespace maxt {

class SendGeoPoseNode : public BT::StatefulActionNode {
public:
    SendGeoPoseNode(const std::string& name, const BT::NodeConfiguration& config,
                    ros::NodeHandle& nh);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}

private:
    void onFix(const sensor_msgs::NavSatFix::ConstPtr& message);

    std::mutex mutex_;
    sensor_msgs::NavSatFix fix_;
    ros::WallTime received_at_;
    ros::WallTime started_;
    bool valid_{false};
    double timeout_{10.0};
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
};

} // namespace maxt
#endif
