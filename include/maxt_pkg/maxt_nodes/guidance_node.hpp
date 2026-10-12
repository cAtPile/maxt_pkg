#ifndef MAXT_NODES_GUIDANCE_HPP
#define MAXT_NODES_GUIDANCE_HPP

#include <maxt_pkg/maxt_nodes.hpp>
#include <maxt_pkg/pid_controller.hpp>
#include <maxt_pkg/GuidanceAction.h>
#include <actionlib/client/simple_action_client.h>
#include <memory>
#include <mutex>

namespace maxt {

// Feedback callbacks own only a shared snapshot, never the BT node itself.
class GuidanceNode : public MavActionNode {
public:
    GuidanceNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);
    ~GuidanceNode() override;
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    using Client = actionlib::SimpleActionClient<maxt_pkg::GuidanceAction>;

    // 运行阶段由 onRunning 推进，action 回调仅更新反馈快照。
    enum class GuidanceState {
        WAIT_SERVER,
        WAIT_FEEDBACK,
        GUIDING,
        REACHED
    };

    GuidanceState guidance_state_{GuidanceState::WAIT_SERVER};
    
    struct FeedbackState {
        std::mutex mutex;
        maxt_pkg::GuidanceFeedback feedback;
        ros::Time received;
        unsigned long sequence{0};
    };

    void sendGoal();
    
    BT::NodeStatus finish(BT::NodeStatus status, const std::string& message);
    std::unique_ptr<Client> client_;
    std::shared_ptr<FeedbackState> feedback_;
    maxt_pkg::GuidanceGoal goal_;
    PIDController pid_u_, pid_v_;
    ros::Time start_time_, last_update_;
    ros::Time tracking_start_;
    double tracking_duration_{0.0};
    bool downward_camera_{false}, tracking_started_{false};
    unsigned long processed_sequence_{0};
    bool goal_sent_{false};
    const std::string action_name_{"/guidance"};
    double bias_u_{0.0}, bias_v_{0.0};
    double tolerance_{5.0}, timeout_{30.0};
    const double feedback_timeout_{0.5};
    double kp_{0.01};
    const double ki_{0.0}, kd_{0.0};
    const double integral_max_{50.0}, max_speed_{0.5};
    double u_to_x_{1.0}, u_to_y_{0.0}, v_to_x_{0.0}, v_to_y_{1.0};
    double hold_z_{0.0}, hold_yaw_{0.0};

};

} // namespace maxt
#endif
