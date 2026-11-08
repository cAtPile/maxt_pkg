#include <maxt_pkg/maxt_nodes/send_geo_pose_node.hpp>
#include <cmath>

namespace maxt {

SendGeoPoseNode::SendGeoPoseNode(const std::string& name,
                               const BT::NodeConfiguration& config, ros::NodeHandle& nh)
    : BT::SyncActionNode(name, config) {
    publisher_ = nh.advertise<sensor_msgs::NavSatFix>("/QRCode_pose/Global_pose", 10);
    subscriber_ = nh.subscribe("/mavros/global_position/global", 1,
                               &SendGeoPoseNode::onFix, this);
}

BT::PortsList SendGeoPoseNode::providedPorts() {
    return {};
}

void SendGeoPoseNode::onFix(const sensor_msgs::NavSatFix::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    fix_ = *message;
    received_at_ = ros::WallTime::now();
    valid_ = message->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX &&
        std::isfinite(message->latitude) && std::isfinite(message->longitude) &&
        std::abs(message->latitude) <= 90.0 && std::abs(message->longitude) <= 180.0;
}

BT::NodeStatus SendGeoPoseNode::tick() {
    sensor_msgs::NavSatFix message;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // 同步 tick 不等待回调；拒绝无效或超过两秒未更新的定位。
        if (!valid_ || (ros::WallTime::now() - received_at_).toSec() > 2.0) {
            ROS_WARN("SendGeoPose: no valid recent GPS, skipping send");
            return BT::NodeStatus::FAILURE;
        }
        message = fix_;
    }
    if (publisher_.getNumSubscribers() == 0) {
        ROS_WARN("SendGeoPose: no receiver connected, skipping send");
        return BT::NodeStatus::FAILURE;
    }
    publisher_.publish(message);
    ROS_INFO("SendGeoPose: sent latitude=%.9f longitude=%.9f altitude=%.3f",
             message.latitude, message.longitude, message.altitude);
    return BT::NodeStatus::SUCCESS;
}

} // namespace maxt
