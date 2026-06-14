/**
 * @brief deliver_node.cpp
 * @details 控制无人机投放物品
 * @TODO: 连接是否可以统一建立
 */
#include <maxt_pkg/maxt_nodes/deliver_node.hpp>
#include <boost/algorithm/string.hpp>

namespace maxt {

// 定义投放机构的servo名称
// TODO：命名写到readme中
const std::vector<std::string> DeliverNode::SERVO_NAMES = {
    "front_left", "front_right", "back_left", "back_right"
};

DeliverNode::DeliverNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav,
                         ros::Publisher& fl_open, ros::Publisher& fl_close,
                         ros::Publisher& fr_open, ros::Publisher& fr_close,
                         ros::Publisher& bl_open, ros::Publisher& bl_close,
                         ros::Publisher& br_open, ros::Publisher& br_close)
    : MavActionNode(name, config, mav)
{
    open_pubs_[0] = &fl_open;  close_pubs_[0] = &fl_close;
    open_pubs_[1] = &fr_open;  close_pubs_[1] = &fr_close;
    open_pubs_[2] = &bl_open;  close_pubs_[2] = &bl_close;
    open_pubs_[3] = &br_open;  close_pubs_[3] = &br_close;
}

BT::PortsList DeliverNode::providedPorts() {
    return {
        BT::InputPort<std::string>("servo_a", "front_left", "primary servo door"),// 主舵机
        BT::InputPort<std::string>("servo_b", "front_right", "fallback servo door"),// 备用舵机
        BT::InputPort<double>("deliver_delay", 2.0, "Delay between open and close (seconds)"),// 投放延迟
        BT::InputPort<double>("connect_delay", 0.2, "Delay for connection establishment (seconds)")// 连接建立延迟
    };
}

BT::NodeStatus DeliverNode::onStart() {
    delivery_phase_ = 0;
    servo_idx_a_ = -1;
    servo_idx_b_ = -1;
    chosen_servo_idx_ = -1;

    getInput<double>("deliver_delay", deliver_delay_);
    getInput<double>("connect_delay", connect_delay_);

    // 解析 servo_a
    std::string name_a, name_b;
    getInput<std::string>("servo_a", name_a);
    boost::trim(name_a);
    getInput<std::string>("servo_b", name_b);
    boost::trim(name_b);

    for (size_t i = 0; i < SERVO_NAMES.size(); i++) {
        if (SERVO_NAMES[i] == name_a) servo_idx_a_ = static_cast<int>(i);
        if (SERVO_NAMES[i] == name_b) servo_idx_b_ = static_cast<int>(i);
    }

    if (servo_idx_a_ < 0) {
        ROS_ERROR("DeliverNode: Invalid servo_a '%s'", name_a.c_str());
        return BT::NodeStatus::FAILURE;
    }
    if (servo_idx_b_ < 0) {
        ROS_ERROR("DeliverNode: Invalid servo_b '%s'", name_b.c_str());
        return BT::NodeStatus::FAILURE;
    }
    if (servo_idx_a_ == servo_idx_b_) {
        ROS_ERROR("DeliverNode: servo_a and servo_b must be different");
        return BT::NodeStatus::FAILURE;
    }

    // 选择投放舵机: a 未投 → a; a 已投 → b
    auto bb = config().blackboard;
    bool delivered_a = false;
    bb->get("is_delivered_" + SERVO_NAMES[servo_idx_a_], delivered_a);

    if (!delivered_a) {
        chosen_servo_idx_ = servo_idx_a_;
    } else {
        chosen_servo_idx_ = servo_idx_b_;
    }

    connect_start_ = ros::Time::now();
    ROS_INFO("DeliverNode: a=%s(delivered=%d) b=%s → chosen=%s",
             SERVO_NAMES[servo_idx_a_].c_str(), delivered_a,
             SERVO_NAMES[servo_idx_b_].c_str(),
             SERVO_NAMES[chosen_servo_idx_].c_str());

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DeliverNode::onRunning() {
    // Phase 0: 等待连接建立
    if (delivery_phase_ == 0) {
        if ((ros::Time::now() - connect_start_).toSec() >= connect_delay_) {
            delivery_phase_ = 1;
        } else {
            return BT::NodeStatus::RUNNING;
        }
    }

    // Phase 1: 打开舱门
    if (delivery_phase_ == 1) {
        std_msgs::Empty msg;
        open_pubs_[chosen_servo_idx_]->publish(msg);
        ROS_INFO("DeliverNode: Opened %s", SERVO_NAMES[chosen_servo_idx_].c_str());

        deliver_start_ = ros::Time::now();
        delivery_phase_ = 2;
        return BT::NodeStatus::RUNNING;
    }

    // Phase 2: 等待投放延迟
    if (delivery_phase_ == 2) {
        if ((ros::Time::now() - deliver_start_).toSec() >= deliver_delay_) {
            ROS_INFO("DeliverNode: Delay %.1fs elapsed, closing mechanism.",
                     deliver_delay_);
            delivery_phase_ = 3;
        } else {
            return BT::NodeStatus::RUNNING;
        }
    }

    // Phase 3: 关闭舱门，标记完成
    if (delivery_phase_ == 3) {
        std_msgs::Empty msg;
        close_pubs_[chosen_servo_idx_]->publish(msg);
        ROS_INFO("DeliverNode: Closed %s", SERVO_NAMES[chosen_servo_idx_].c_str());

        config().blackboard->set("is_delivered_" + SERVO_NAMES[chosen_servo_idx_], true);
        ROS_INFO("DeliverNode: Marked is_delivered_%s = true",
                 SERVO_NAMES[chosen_servo_idx_].c_str());

        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void DeliverNode::onHalted() {
    ROS_WARN("DeliverNode: Halted during delivery!");
}

} // namespace maxt
