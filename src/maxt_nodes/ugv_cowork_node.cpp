#include <maxt_pkg/maxt_nodes/ugv_cowork_node.hpp>
#include <cmath>

namespace maxt {

UgvCoworkNode::UgvCoworkNode(const std::string& name,
                           const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav) {}

UgvCoworkNode::~UgvCoworkNode() {
    cancelGoal();
}

BT::PortsList UgvCoworkNode::providedPorts() {
    return {
        BT::InputPort<std::string>("action_name", "/ugv_mover", "UGV Action server"),
        BT::InputPort<double>("timeout", 60.0, "server connection and arrival timeout (wall seconds)"),
        BT::InputPort<bool>("follow_drone", false, "update UGV target from drone XY without controlling drone"),
        BT::InputPort<double>("update_period", 0.2, "minimum target update interval (wall seconds)"),
        BT::InputPort<double>("target_tolerance", 0.2, "drone XY movement needed to replace UGV target (m)"),
        BT::InputPort<double>("settle_time", 1.0, "minimum age of final target before accepting arrival (wall seconds)"),
        BT::OutputPort<std::string>("message", "completion reason")
    };
}

BT::NodeStatus UgvCoworkNode::onStart() {
    std::string action_name;
    getInput("action_name", action_name);
    getInput("timeout", timeout_);
    getInput("follow_drone", follow_drone_);
    getInput("update_period", update_period_);
    getInput("target_tolerance", target_tolerance_);
    getInput("settle_time", settle_time_);
    started_ = ros::WallTime::now();
    goal_sent_ = false;
    // 使用核心的 AsyncSpinner 处理回调；每次启动建立独立客户端。
    client_.reset(new Client(action_name, false));

    const auto position = mav_.getCurrentPose().pose.position;
    goal_.target_x = position.x;
    goal_.target_y = position.y;

    // 保留原独立任务的悬停行为；并行跟随模式下飞控完全交给 Guidance。
    if (!follow_drone_) {
        mavros_msgs::PositionTarget hold;
        hold.header.frame_id = "map";
        hold.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        hold.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        hold.position = position;
        hold.yaw = mav_.get_current_yaw();
        mav_.setRawTarget(hold);
        mav_.setSetpointMode(SetpointMode::RAW_CTRL);
    }
    setOutput("message", std::string("WAIT_SERVER"));
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus UgvCoworkNode::onRunning() {
    if ((ros::WallTime::now() - started_).toSec() >= timeout_) {
        return finish(BT::NodeStatus::FAILURE, "TIMEOUT");
    }
    if (!goal_sent_) {
        if (client_->isServerConnected()) {
            const auto position = mav_.getCurrentPose().pose.position;
            sendGoal(follow_drone_ ? position.x : goal_.target_x,
                     follow_drone_ ? position.y : goal_.target_y);
        }
        return BT::NodeStatus::RUNNING;
    }

    const auto state = client_->getState();
    if (state.isDone() && state != actionlib::SimpleClientGoalState::SUCCEEDED) {
        return finish(BT::NodeStatus::FAILURE, "UGV_FAILED: " + state.toString());
    }
    const auto result = client_->getResult();
    if (state.isDone() && (!result || !result->succeed)) {
        return finish(BT::NodeStatus::FAILURE, "UGV_FAILED: unsuccessful result");
    }
    if (follow_drone_) {
        const auto position = mav_.getCurrentPose().pose.position;
        const double movement = std::hypot(position.x - goal_.target_x,
                                           position.y - goal_.target_y);
        const double age = (ros::WallTime::now() - goal_updated_).toSec();
        // 先检查新位置，旧目标的成功不能结束正在移动的目标跟随。
        if (movement > target_tolerance_) {
            if (age >= update_period_) {
                sendGoal(position.x, position.y);
            }
            return BT::NodeStatus::RUNNING;
        }
        if (age < settle_time_) {
            return BT::NodeStatus::RUNNING;
        }
    }
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED && result && result->succeed) {
        return finish(BT::NodeStatus::SUCCESS, "UGV_REACHED");
    }
    return BT::NodeStatus::RUNNING;
}

void UgvCoworkNode::sendGoal(double x, double y) {
    goal_.target_x = x;
    goal_.target_y = y;
    // SimpleActionClient 改为跟踪新 goal；旧 goal 由服务端正常抢占。
    client_->sendGoal(goal_);
    goal_sent_ = true;
    goal_updated_ = ros::WallTime::now();
    setOutput("message", std::string("WAIT_UGV"));
    ROS_INFO("UgvCowork: target (%.2f, %.2f)", goal_.target_x, goal_.target_y);
}

void UgvCoworkNode::cancelGoal() {
    // 仅取消本节点仍在执行的目标，不影响其他客户端的小车任务。
    if (client_ && goal_sent_ && !client_->getState().isDone()) {
        client_->cancelGoal();
    }
    goal_sent_ = false;
}

BT::NodeStatus UgvCoworkNode::finish(BT::NodeStatus status, const std::string& message) {
    cancelGoal();
    if (!follow_drone_) {
        mav_.setSetpointMode(SetpointMode::HEARTBEAT);
    }
    setOutput("message", message);
    ROS_INFO("UgvCowork: %s", message.c_str());
    return status;
}

void UgvCoworkNode::onHalted() {
    finish(BT::NodeStatus::FAILURE, "HALTED");
}

} // namespace maxt
