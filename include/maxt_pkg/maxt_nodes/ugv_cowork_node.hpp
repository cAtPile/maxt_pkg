#ifndef MAXT_UGV_COWORK_NODE_HPP
#define MAXT_UGV_COWORK_NODE_HPP

#include <actionlib/client/simple_action_client.h>
#include <maxt_pkg/maxt_nodes.hpp>
#include <maxt_pkg/ugv_moverAction.h>
#include <memory>

namespace maxt {

// 向小车发送无人机 XY；跟随模式更新目标，不参与无人机飞行控制。
class UgvCoworkNode : public MavActionNode {
public:
    UgvCoworkNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);
    ~UgvCoworkNode() override;
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    using Client = actionlib::SimpleActionClient<maxt_pkg::ugv_moverAction>;
    BT::NodeStatus finish(BT::NodeStatus status, const std::string& message);
    void cancelGoal();
    void sendGoal(double x, double y);

    std::unique_ptr<Client> client_;
    maxt_pkg::ugv_moverGoal goal_;
    ros::WallTime started_;
    double timeout_{60.0};
    bool goal_sent_{false};
    bool follow_drone_{false};
    double update_period_{0.2};
    double target_tolerance_{0.2};
    double settle_time_{1.0};
    ros::WallTime goal_updated_;
};

} // namespace maxt
#endif
