#include <maxt_pkg/maxt_nodes/send_geo_pose_node.hpp>
#include <cmath>

namespace maxt {

SendGeoPoseNode::SendGeoPoseNode(const std::string& name,
                               const BT::NodeConfiguration& config, ros::NodeHandle& nh)
    : BT::StatefulActionNode(name, config) {
    publisher_ = nh.advertise<sensor_msgs::NavSatFix>("QRCode_pose/Global_pose", 10);
    subscriber_ = nh.subscribe("/mavros/global_position/global", 1,
                               &SendGeoPoseNode::onFix, this);
}

BT::PortsList SendGeoPoseNode::providedPorts() {
    return {BT::InputPort<double>("timeout", 10.0, "Wait for GPS and receiver (seconds)")};
}

void SendGeoPoseNode::onFix(const sensor_msgs::NavSatFix::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    fix_ = *message;
    received_at_ = ros::WallTime::now();
    valid_ = message->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX &&
        std::isfinite(message->latitude) && std::isfinite(message->longitude) &&
        std::abs(message->latitude) <= 90.0 && std::abs(message->longitude) <= 180.0;
}

BT::NodeStatus SendGeoPoseNode::onStart() {
    if (!getInput("timeout", timeout_) || !std::isfinite(timeout_) || timeout_ <= 0.0) {
        ROS_ERROR("SendGeoPose: timeout must be a positive finite number");
        return BT::NodeStatus::FAILURE;
    }
    started_ = ros::WallTime::now();
    return onRunning();
}

BT::NodeStatus SendGeoPoseNode::onRunning() {
    if ((ros::WallTime::now() - started_).toSec() >= timeout_) {
        ROS_ERROR("SendGeoPose: timeout waiting for valid GPS or a receiver");
        return BT::NodeStatus::FAILURE;
    }
    sensor_msgs::NavSatFix message;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // 每次执行必须收到新的定位，避免发送上次任务缓存的位置。
        if (!valid_ || received_at_ < started_) {
            return BT::NodeStatus::RUNNING;
        }
        message = fix_;
    }
    if (publisher_.getNumSubscribers() == 0) {
        return BT::NodeStatus::RUNNING;
    }
    publisher_.publish(message);
    ROS_INFO("SendGeoPose: sent latitude=%.9f longitude=%.9f altitude=%.3f",
             message.latitude, message.longitude, message.altitude);
    return BT::NodeStatus::SUCCESS;
}

} // namespace maxt
