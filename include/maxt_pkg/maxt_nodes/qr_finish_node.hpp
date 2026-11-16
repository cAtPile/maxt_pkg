#pragma once
#include <maxt_pkg/maxt_nodes.hpp>
#include <maxt_pkg/qr_survey_support.hpp>
namespace maxt {
class QrFinish : public MavActionNode {
    std::shared_ptr<QrSurveyContext> context_;
    AsyncQrService<uav_qr_mission::FinishSurvey> call_;
    mavros_msgs::PositionTarget hold_;
    ros::SteadyTime started_;
    std::string drop_;
public:
    QrFinish(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav,
             std::shared_ptr<QrSurveyContext> context)
        : MavActionNode(name, config, mav), context_(context) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override { mav_.setSetpointMode(SetpointMode::SLOW_DOWN); }
};
} // namespace maxt
