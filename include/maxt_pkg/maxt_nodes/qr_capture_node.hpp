#pragma once
#include <maxt_pkg/maxt_nodes.hpp>
#include <maxt_pkg/qr_survey_support.hpp>
namespace maxt {
class QrCapture : public MavActionNode {
    ros::NodeHandle params_;
    std::shared_ptr<QrSurveyContext> context_;
    AsyncQrService<uav_qr_mission::CaptureQr> call_, cancel_call_;
    uav_qr_mission::CaptureQr service_;
    mavros_msgs::PositionTarget hold_;
    ros::SteadyTime started_, stable_since_, capture_started_;
    double settle_{1}, stable_time_{0.4}, speed_{0.2}, tilt_{10}, tolerance_{0.2};
    double settle_timeout_{20}, capture_timeout_{3};
    bool requested_{false}, accepted_{false};
    bool stable() const;
    void cancel();
public:
    QrCapture(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav,
              ros::NodeHandle nh, std::shared_ptr<QrSurveyContext> context);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};
} // namespace maxt
