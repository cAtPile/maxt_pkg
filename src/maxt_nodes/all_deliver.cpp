#include <maxt_pkg/maxt_nodes/all_deliver_node.hpp>

namespace maxt {

AllDeliverNode::AllDeliverNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav,
                               ros::Publisher& all_open, ros::Publisher& all_close)
    : MavActionNode(name, config, mav), all_open_pub_(all_open), all_close_pub_(all_close) {
}

BT::PortsList AllDeliverNode::providedPorts() {
    return {
        BT::InputPort<double>("deliver_delay", 2.0, "Delay between open and close (seconds)"),
        BT::InputPort<double>("connect_delay", 0.2, "Delay for connection establishment (seconds)")
    };
}

BT::NodeStatus AllDeliverNode::onStart() {
    delivery_phase_ = 0;

    getInput<double>("deliver_delay", deliver_delay_);
    getInput<double>("connect_delay", connect_delay_);

    connect_start_ = ros::Time::now();
    ROS_INFO("AllDeliverNode: Waiting %.1fs for connection, will open all doors",
             connect_delay_);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AllDeliverNode::onRunning() {
    // Phase 0: 等待连接建立
    if (delivery_phase_ == 0) {
        if ((ros::Time::now() - connect_start_).toSec() >= connect_delay_) {
            delivery_phase_ = 1;
        } else {
            return BT::NodeStatus::RUNNING;
        }
    }

    // Phase 1: 打开全部舱门
    if (delivery_phase_ == 1) {
        std_msgs::Empty msg;
        all_open_pub_.publish(msg);
        ROS_INFO("AllDeliverNode: Opened all doors");

        deliver_start_ = ros::Time::now();
        delivery_phase_ = 2;
        return BT::NodeStatus::RUNNING;
    }

    // Phase 2: 等待投放延迟
    if (delivery_phase_ == 2) {
        if ((ros::Time::now() - deliver_start_).toSec() >= deliver_delay_) {
            ROS_INFO("AllDeliverNode: Delay %.1fs elapsed, closing all mechanisms.",
                     deliver_delay_);
            delivery_phase_ = 3;
        } else {
            return BT::NodeStatus::RUNNING;
        }
    }

    // Phase 3: 关闭全部舱门，标记完成
    if (delivery_phase_ == 3) {
        std_msgs::Empty msg;
        all_close_pub_.publish(msg);
        ROS_INFO("AllDeliverNode: Closed all doors");

        config().blackboard->set("is_delivered_front_left", true);
        config().blackboard->set("is_delivered_front_right", true);
        config().blackboard->set("is_delivered_back_left", true);
        config().blackboard->set("is_delivered_back_right", true);
        ROS_INFO("AllDeliverNode: Marked all doors as delivered");

        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void AllDeliverNode::onHalted() {
    ROS_WARN("AllDeliverNode: Halted during delivery!");
}

} // namespace maxt
