#include <maxt_pkg/maxt_nodes/guidance_node.hpp>
#include <cmath>

namespace maxt {

GuidanceNode::GuidanceNode(const std::string& name,
                           const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav) {}

GuidanceNode::~GuidanceNode() {
    if (goal_sent_ && client_) {
        client_->cancelGoal();
    }
}

BT::PortsList GuidanceNode::providedPorts() {
    return {
        BT::InputPort<std::string>("target_id"),
        BT::InputPort<double>("tolerance", 5.0, "per-axis error tolerance"),
        BT::InputPort<double>("timeout", 30.0, "whole operation timeout in seconds"),
        BT::OutputPort<std::string>("message", "local completion or server result")
    };
}

BT::NodeStatus GuidanceNode::onStart() {

    if (!getInput("target_id", goal_.target_id) || goal_.target_id.empty()) {
        return finish(BT::NodeStatus::FAILURE, "FAIL: missing target_id");
    }//请分离getinput和校验
    getInput("tolerance", tolerance_);
    getInput("timeout", timeout_);
    if (tolerance_ < 0.0 || timeout_ <= 0.0) {
        return finish(BT::NodeStatus::FAILURE, "FAIL: invalid limits");
    }

    pid_u_ = PIDController(kp_, ki_, kd_, integral_max_, max_speed_);
    pid_v_ = PIDController(kp_, ki_, kd_, integral_max_, max_speed_);
    hold_z_ = mav_.getCurrentPose().pose.position.z;
    hold_yaw_ = mav_.get_current_yaw();

    start_time_ = ros::Time::now();
    last_update_ = start_time_;
    processed_sequence_ = 0;
    goal_sent_ = false;
    guidance_state_ = GuidanceState::WAIT_SERVER;

    feedback_ = std::make_shared<FeedbackState>();
    client_.reset(new Client(action_name_, false));
    mav_.setSetpointMode(SetpointMode::HEARTBEAT);
    setOutput("message", std::string("RUNNING"));
    if (client_->isServerConnected()) {
        sendGoal();
    }
    return BT::NodeStatus::RUNNING;
}

void GuidanceNode::sendGoal() {
    auto snapshot = feedback_;
    client_->sendGoal(goal_, Client::SimpleDoneCallback(), Client::SimpleActiveCallback(),
        [snapshot](const maxt_pkg::GuidanceFeedbackConstPtr& feedback) {
            std::lock_guard<std::mutex> lock(snapshot->mutex);
            snapshot->feedback = *feedback;
            snapshot->received = ros::Time::now();
            ++snapshot->sequence;
        });
    goal_sent_ = true;
    guidance_state_ = GuidanceState::WAIT_FEEDBACK;
    
}

BT::NodeStatus GuidanceNode::onRunning() {

    const ros::Time now = ros::Time::now();
    if ((now - start_time_).toSec() >= timeout_) {
        return finish(BT::NodeStatus::FAILURE, "TIMEOUT");
    }
    maxt_pkg::GuidanceFeedback feedback;
    ros::Time received;

    unsigned long sequence;
    {
        std::lock_guard<std::mutex> lock(feedback_->mutex);
        feedback = feedback_->feedback;
        received = feedback_->received;
        sequence = feedback_->sequence;
    }
    const auto action_state = client_->getState();

    // 全局终止条件优先于状态机，避免旧反馈覆盖服务端失败。
    if (goal_sent_ && action_state.isDone() &&
        action_state != actionlib::SimpleClientGoalState::SUCCEEDED) {
        auto result = client_->getResult();
        return finish(BT::NodeStatus::FAILURE,
                      result ? result->message : action_state.toString());
    }
    const bool fresh = sequence > 0 && (now - received).toSec() <= feedback_timeout_;
    const bool visible = fresh && feedback.target_visible;

    switch (guidance_state_) {

        case GuidanceState::WAIT_SERVER:
            // 等待连接，不阻塞行为树；sendGoal 切换到 WAIT_FEEDBACK。
            if (client_->isServerConnected()) {
                sendGoal();
            }
            return BT::NodeStatus::RUNNING;

        case GuidanceState::WAIT_FEEDBACK:
            // 首次等待和丢失目标后的等待共用此状态。
            if (!visible) {
                if (action_state.isDone()) {
                    return finish(BT::NodeStatus::FAILURE,
                                  "FAIL: no visible fresh feedback");
                }
                last_update_ = now;
                return BT::NodeStatus::RUNNING;
            }
            guidance_state_ = GuidanceState::GUIDING;
            // fall through: 首条有效反馈在当前 tick 进入引导处理。

        case GuidanceState::GUIDING: {
            if (!visible) {
                mav_.setSetpointMode(SetpointMode::HEARTBEAT);
                pid_u_.reset();
                pid_v_.reset();
                last_update_ = now;
                guidance_state_ = GuidanceState::WAIT_FEEDBACK;
                if (action_state.isDone()) {
                    return finish(BT::NodeStatus::FAILURE,
                                  "FAIL: no visible fresh feedback");
                }
                return BT::NodeStatus::RUNNING;
            }

            if (std::abs(feedback.error_u - bias_u_) <= tolerance_ &&
                std::abs(feedback.error_v - bias_v_) <= tolerance_) {
                guidance_state_ = GuidanceState::REACHED;
            }

            if (action_state.isDone()) {
                auto result = client_->getResult();
                return finish(BT::NodeStatus::FAILURE,
                              result ? "FAIL: not aligned; " + result->message
                                     : "FAIL: not aligned");
            }

            // 每条反馈只计算一次 PID，定时器持续发布最新设点。
            if (sequence == processed_sequence_) {
                return BT::NodeStatus::RUNNING;
            }

            const double dt = (received - last_update_).toSec();
            const double u = pid_u_.update(bias_u_, feedback.error_u, dt);
            const double v = pid_v_.update(bias_v_, feedback.error_v, dt);
            double vx = u_to_x_ * u + v_to_x_ * v;
            double vy = u_to_y_ * u + v_to_y_ * v;
            const double speed = std::hypot(vx, vy);
            if (speed > max_speed_) {
                vx *= max_speed_ / speed;
                vy *= max_speed_ / speed;
            }

            mavros_msgs::PositionTarget target;
            target.header.frame_id = "map";
            target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            // 水平速度控制，保持开始时的高度和偏航。
            target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                mavros_msgs::PositionTarget::IGNORE_PY |
                mavros_msgs::PositionTarget::IGNORE_VZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            target.velocity.x = vx;
            target.velocity.y = vy;
            target.position.z = hold_z_;
            target.yaw = hold_yaw_;
            mav_.setRawTarget(target);
            mav_.setSetpointMode(SetpointMode::RAW_CTRL);
            processed_sequence_ = sequence;
            last_update_ = received;
            return BT::NodeStatus::RUNNING;
        }

        case GuidanceState::REACHED:
            return finish(BT::NodeStatus::SUCCESS, "SUCCEED");
    }
    return finish(BT::NodeStatus::FAILURE, "FAIL: undesigned failure guidance state");
}

BT::NodeStatus GuidanceNode::finish(BT::NodeStatus status, const std::string& message) {
    if (client_ && goal_sent_) {
        client_->cancelGoal();
        goal_sent_ = false;
    }
    mav_.setSetpointMode(SetpointMode::HEARTBEAT);
    setOutput("message", message);
    ROS_INFO("GuidanceNode: %s", message.c_str());
    return status;
}

void GuidanceNode::onHalted() {
    finish(BT::NodeStatus::FAILURE, "HALTED");
}

} // namespace maxt
