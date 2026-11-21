#ifndef MAXT_NODES_NAV_AVOIDE_HPP
#define MAXT_NODES_NAV_AVOIDE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/transform_listener.h>

namespace maxt {

class MavKit;

class NavAvoideNode : public BT::StatefulActionNode {
public:
    NavAvoideNode(const std::string& name, const BT::NodeConfiguration& config,
                  MavKit& mav, const ros::NodeHandle& nh);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void command(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    void cloud(const sensor_msgs::PointCloud::ConstPtr& msg);

    MavKit& mav_;
    ros::CallbackQueue queue_;
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Publisher goals_, targets_, planner_cloud_;
    ros::Subscriber commands_, lidar_cloud_;
    geometry_msgs::PoseStamped goal_;
    ros::WallTime start_;
    double timeout_, tolerance_;
    bool sent_{false}, endpoint_commanded_{false};
};

} // namespace maxt

#endif
