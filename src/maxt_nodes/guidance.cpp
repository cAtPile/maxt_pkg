#include <maxt_pkg/maxt_nodes/guidance_node.hpp>
#include <cmath>

namespace maxt {

// 视觉引导节点：向 /guidance 请求目标的视觉误差，用两个 PID 生成水平速度。
// 行为树每次 tick 推进一小步，不在这里阻塞等待服务器或反馈。
// 正常流程：WAIT_SERVER（等待连接）→ WAIT_FEEDBACK（等待有效反馈）
//          → GUIDING（调整位置）→ REACHED（下一次 tick 返回成功）。
// 引导中目标丢失或反馈过期时，退回 WAIT_FEEDBACK；整段任务受 timeout 限制。
GuidanceNode::GuidanceNode(const std::string& name,
                           const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav) {}

GuidanceNode::~GuidanceNode() {
    if (goal_sent_ && client_) {
        client_->cancelGoal();
    }
}

BT::PortsList GuidanceNode::providedPorts() {
    // 行为树端口：target_id 指定目标；tolerance 是 u、v 各轴允许的误差，
    return {
        BT::InputPort<std::string>("target_id"),
        BT::InputPort<double>("tolerance", 5.0, "per-axis error tolerance"),
        BT::InputPort<double>("timeout", 30.0, "whole operation timeout in seconds"),
        BT::InputPort<double>("tracking_duration", 0.0, "continuous tracking seconds; zero aligns once"),
        BT::InputPort<bool>("downward_camera", false, "PX4 downward camera axes mapped with vehicle yaw"),
        BT::InputPort<double>("kp", 0.01, "pixel error proportional gain"),
        BT::OutputPort<std::string>("message", "local completion or server result")
    };
}

BT::NodeStatus GuidanceNode::onStart() {

    // 节点首次被 tick时执行：读取输入并校验参数。
    if (!getInput("target_id", goal_.target_id) || goal_.target_id.empty()) {
        return finish(BT::NodeStatus::FAILURE, "FAIL: missing target_id");
    }//TODO:请分离getinput和校验
    getInput("tolerance", tolerance_);
    getInput("timeout", timeout_);
    getInput("tracking_duration", tracking_duration_);
    getInput("downward_camera", downward_camera_);
    getInput("kp", kp_);
    tracking_started_ = false;
    if (!std::isfinite(tolerance_) || !std::isfinite(timeout_) ||
        !std::isfinite(tracking_duration_) || !std::isfinite(kp_) ||
        tolerance_ < 0.0 || timeout_ <= 0.0 || tracking_duration_ < 0.0 ||
        tracking_duration_ >= timeout_ || kp_ <= 0.0) {
        return finish(BT::NodeStatus::FAILURE, "FAIL: invalid limits");
    }

    //PID
    pid_u_ = PIDController(kp_, ki_, kd_, integral_max_, max_speed_);
    pid_v_ = PIDController(kp_, ki_, kd_, integral_max_, max_speed_);

    //维持
    hold_z_ = mav_.getCurrentPose().pose.position.z;
    hold_yaw_ = mav_.get_current_yaw();

    //超时
    start_time_ = ros::Time::now();
    last_update_ = start_time_;

    // sequence 是收到反馈的计数，processed_sequence_ 是已用于 PID 的反馈编号。？？
    processed_sequence_ = 0;
    goal_sent_ = false;
    guidance_state_ = GuidanceState::WAIT_SERVER;

    // 每次启动使用新的反馈快照，避免上次任务的回调污染本次任务。
    // Client 的 false 表示不创建内部 spin 线程，回调由外部 ROS spinner 处理。
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
    // 回调只捕获 shared_ptr 快照，不捕获 this；即使节点销毁，
    // 回调持有的数据仍然有效。回调只存反馈，控制和状态切换由 BT 线程处理。
    auto snapshot = feedback_;
    client_->sendGoal(goal_, Client::SimpleDoneCallback(), Client::SimpleActiveCallback(),
        [snapshot](const maxt_pkg::GuidanceFeedbackConstPtr& feedback) {
            // 与 onRunning 的读取共用一把锁，保证数据、时间和编号一致。
            std::lock_guard<std::mutex> lock(snapshot->mutex);
            snapshot->feedback = *feedback;
            // 记录本地接收时间；GuidanceFeedback 本身没有时间戳。
            snapshot->received = ros::Time::now();
            ++snapshot->sequence;
        });
    goal_sent_ = true;
    
}

BT::NodeStatus GuidanceNode::onRunning() {

    // 超时
    const ros::Time now = ros::Time::now();
    if ((now - start_time_).toSec() >= timeout_) {
        return finish(BT::NodeStatus::FAILURE, "TIMEOUT");
    }
    maxt_pkg::GuidanceFeedback feedback;
    ros::Time received;

    unsigned long sequence;
    {
        // 锁内只复制一份一致的快照，离开作用域自动解锁；后续计算不占锁。
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

    //新鲜
    const bool fresh = sequence > 0 && (now - received).toSec() <= feedback_timeout_;

    //可视
    const bool visible = fresh && feedback.target_visible;

    switch (guidance_state_) {

        case GuidanceState::WAIT_SERVER:
            // 等待连接
            if (client_->isServerConnected()) {
                sendGoal();
                guidance_state_ = GuidanceState::WAIT_FEEDBACK;
            }
            return BT::NodeStatus::RUNNING;

        case GuidanceState::WAIT_FEEDBACK:
            // 首次等待和丢失目标后的等待共用此状态。
            if (!visible) {
                if (action_state.isDone()) {
                    return finish(BT::NodeStatus::FAILURE,
                                  "FAIL: no visible fresh feedback");
                }
                // 等待期间刷新基准时间，避免把长时间等待全部计入 PID 的 dt。
                last_update_ = now;
                return BT::NodeStatus::RUNNING;
            }
            guidance_state_ = GuidanceState::GUIDING;
            // fall through: 首条有效反馈在当前 tick 进入引导处理。

        case GuidanceState::GUIDING: {
            if (!tracking_started_) {
                tracking_start_ = now;
                tracking_started_ = true;
            }
            if (tracking_duration_ > 0.0 &&
                (now - tracking_start_).toSec() >= tracking_duration_) {
                return finish(BT::NodeStatus::SUCCESS, "TRACKING_COMPLETE");
            }
            if (!visible) {
                // 目标丢失：停止使用原来的速度设点，切回当前位置心跳，
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

            // 对其检查
            if (tracking_duration_ == 0.0 &&
                std::abs(feedback.error_u - bias_u_) <= tolerance_ &&
                std::abs(feedback.error_v - bias_v_) <= tolerance_) {
                guidance_state_ = GuidanceState::REACHED;
            }

            // 
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
            if (downward_camera_) {
                // Camera right = body -Y; camera down = body -X.
                // PID outputs are negative pixel errors. Rotate body FLU to ENU.
                const double yaw = mav_.get_current_yaw();
                vx = std::cos(yaw) * v - std::sin(yaw) * u;
                vy = std::sin(yaw) * v + std::cos(yaw) * u;
            }
            const double speed = std::hypot(vx, vy);
            if (speed > max_speed_) {
                vx *= max_speed_ / speed;
                vy *= max_speed_ / speed;
            }

            mavros_msgs::PositionTarget target;
            target.header.frame_id = "map";
            target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
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
            // 保存最新原始设点，并让 MavKit 定时器持续发布；本函数不直接发布。
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
    // 所有正常成功、失败和中断共用的收尾：取消请求、恢复心跳、输出原因。
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
