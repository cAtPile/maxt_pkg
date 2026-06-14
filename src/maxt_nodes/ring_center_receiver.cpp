#include <maxt_pkg/maxt_nodes/ring_center_receiver_node.hpp>

namespace maxt {

RingCenterReceiverNode::RingCenterReceiverNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav),
      ring_locked_(false),
      ring_center_x_(0.0),
      ring_center_y_(0.0),
      sample_count_(0) {
}

BT::PortsList RingCenterReceiverNode::providedPorts() {
    return {
        BT::InputPort<double>("timeout", 10.0, "Timeout for ring center detection"),
        BT::InputPort<double>("offset", 0.7, "Offset distance for front/back waypoints"),
        BT::InputPort<double>("bias", 0.1, "x——bias")
    };
}

void RingCenterReceiverNode::ringCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(ring_mutex_);
    if (ring_locked_) {
        return;
    }

    if (skip_count_ < SKIP_SAMPLES) {
        skip_count_++;
        ROS_INFO("RingCenterReceiver: Skipping stale frame %d/%d (x = %.2f, y = %.2f)",
                 skip_count_, SKIP_SAMPLES, msg->pose.position.x, msg->pose.position.y);
        return;
    }

    x_samples_.push_back(msg->pose.position.x);
    y_samples_.push_back(msg->pose.position.y);
    sample_count_++;

    ROS_INFO("RingCenterReceiver: Collecting sample %d/%d (x = %.2f, y = %.2f)",
             (int)sample_count_, MAX_SAMPLES, msg->pose.position.x, msg->pose.position.y);

    if (sample_count_ >= MAX_SAMPLES) {
        double sum_x = 0.0, sum_y = 0.0;
        for (size_t i = 0; i < x_samples_.size(); i++) {
            sum_x += x_samples_[i];
            sum_y += y_samples_[i];
        }

        ring_center_x_ = sum_x / x_samples_.size();
        ring_center_y_ = sum_y / y_samples_.size();

        ring_locked_ = true;

        ROS_INFO("RingCenterReceiver: Ring center locked (x = %.2f, y = %.2f) after %d samples",
                 ring_center_x_, ring_center_y_, (int)sample_count_);

        x_samples_.clear();
        y_samples_.clear();
    }
}

BT::NodeStatus RingCenterReceiverNode::onStart() {
    ring_locked_ = false;
    ring_center_x_ = 0.0;
    ring_center_y_ = 0.0;
    sample_count_ = 0;
    skip_count_ = 0;
    x_samples_.clear();
    y_samples_.clear();
    start_time_ = ros::Time::now();

    ros::NodeHandle nh;
    ring_sub_ = nh.subscribe("/ring_center", 1, &RingCenterReceiverNode::ringCallback, this);

    getInput<double>("timeout", timeout_);
    getInput<double>("offset", offset_);
    getInput<double>("bias", bias_);

    ROS_INFO("RingCenterReceiver: Collecting ring center samples...");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RingCenterReceiverNode::onRunning() {

    if ((ros::Time::now() - start_time_).toSec() > timeout_) {
        ROS_ERROR("RingCenterReceiver: Timeout! Ring center detection failed.");
        ring_sub_.shutdown();
        return BT::NodeStatus::FAILURE;
    }

    if (!ring_locked_) {
        return BT::NodeStatus::RUNNING;
    }

    auto blackboard = config().blackboard;

    blackboard->set("ring_center_x", ring_center_x_);
    blackboard->set("ring_center_y", ring_center_y_);
    blackboard->set("ring_center_x_front", ring_center_x_ + bias_);
    blackboard->set("ring_center_y_front", ring_center_y_ + offset_);
    blackboard->set("ring_center_x_back",  ring_center_x_ + bias_);
    blackboard->set("ring_center_y_back",  ring_center_y_ - offset_);

    ROS_INFO("RingCenterReceiver: Written to blackboard - "
             "center(%.2f, %.2f) front(%.2f, %.2f) back(%.2f, %.2f) distance=%.2f",
             ring_center_x_, ring_center_y_,
             ring_center_x_ + offset_, ring_center_y_ + offset_,
             ring_center_x_ - offset_, ring_center_y_ - offset_, offset_);

    ring_sub_.shutdown();
    return BT::NodeStatus::SUCCESS;
}

void RingCenterReceiverNode::onHalted() {
    ROS_WARN("RingCenterReceiver: Halted!");
    ring_sub_.shutdown();
}

} // namespace maxt
