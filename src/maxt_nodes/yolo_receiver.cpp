#include <maxt_pkg/maxt_nodes/yolo_receiver_node.hpp>
#include <std_srvs/Empty.h>
#include <boost/algorithm/string.hpp>

namespace maxt {

YoloReceiverNode::YoloReceiverNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav),
      received_(false),
      yolo_detecting_(false)
{
}

BT::PortsList YoloReceiverNode::providedPorts() {
    return {
        BT::InputPort<double>("timeout", 10.0, "Timeout in seconds")
    };
}

void YoloReceiverNode::yoloCallback(const maxt_pkg::StringStamped::ConstPtr& msg) {
    {
        std::lock_guard<std::mutex> lock(yolo_mutex_);
        yolo_data_ = msg->data;
    }
    received_ = true;

    ROS_INFO("YoloReceiver: Received YOLO detection results:");
    for (auto& object : yolo_data_) {
        boost::trim(object);
        ROS_INFO("  - %s", object.c_str());
    }
}

BT::NodeStatus YoloReceiverNode::onStart() {
    received_ = false;
    yolo_data_.clear();

    start_time_ = ros::Time::now();

    ros::NodeHandle nh;
    toggle_yolo_detect_client = nh.serviceClient<std_srvs::Empty>("/toggle_yolo_detect");
    yolo_sub_ = nh.subscribe("/yolo_detect", 1, &YoloReceiverNode::yoloCallback, this);

    std_srvs::Empty empty;
    if (toggle_yolo_detect_client.call(empty)) {
        yolo_detecting_ = true;
        ROS_INFO("YoloReceiver: Started YOLO detection");
    } else {
        ROS_ERROR("YoloReceiver: Failed to start YOLO detection");
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus YoloReceiverNode::onRunning() {
    double timeout;
    getInput<double>("timeout", timeout);

    if (received_) {
        config().blackboard->set("current_object", yolo_data_);

        std_srvs::Empty empty;
        if (yolo_detecting_) {
            toggle_yolo_detect_client.call(empty);
            yolo_detecting_ = false;
        }
        yolo_sub_.shutdown();

        ROS_INFO("YoloReceiver: Stored %zu objects to blackboard current_object", yolo_data_.size());
        return BT::NodeStatus::SUCCESS;
    }

    if ((ros::Time::now() - start_time_).toSec() > timeout) {
        ROS_WARN("YoloReceiver: Timeout after %.2f seconds", timeout);
        std_srvs::Empty empty;
        if (yolo_detecting_) {
            toggle_yolo_detect_client.call(empty);
            yolo_detecting_ = false;
        }
        yolo_sub_.shutdown();
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void YoloReceiverNode::onHalted() {
    std_srvs::Empty empty;
    if (yolo_detecting_) {
        toggle_yolo_detect_client.call(empty);
        yolo_detecting_ = false;
    }
    yolo_sub_.shutdown();
    ROS_INFO("YoloReceiver: Halted");
}

} // namespace maxt
