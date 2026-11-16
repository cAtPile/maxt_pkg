#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <maxt_pkg/qr_survey_support.hpp>
namespace maxt {
class QrSurveySetup : public BT::StatefulActionNode {
    ros::NodeHandle params_;
    std::shared_ptr<QrSurveyContext> context_;
    AsyncQrService<uav_qr_mission::SetTask> call_;
    uav_qr_mission::SetTask service_;
    RegionWaypoints points_;
    double survey_height_{5}, drop_height_{5};
    ros::SteadyTime started_, retry_at_;
    bool accepted_{false};
public:
    QrSurveySetup(const std::string& name, const BT::NodeConfiguration& config,
                  ros::NodeHandle nh, std::shared_ptr<QrSurveyContext> context);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}
};
} // namespace maxt
