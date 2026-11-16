#include <maxt_pkg/maxt_nodes/qr_finish_node.hpp>
namespace maxt {
BT::PortsList QrFinish::providedPorts() {
    return {BT::InputPort<std::string>("mission_id"), BT::InputPort<std::string>("drop_target_id"),
            BT::OutputPort<double>("latitude"), BT::OutputPort<double>("longitude")};
}
BT::NodeStatus QrFinish::onStart() {
    uav_qr_mission::FinishSurvey service;
    if (!getInput("mission_id", service.request.mission_id) ||
        !getInput("drop_target_id", drop_) || !mav_.hasFreshNavigationData())
        return BT::NodeStatus::FAILURE;
    hold_ = qrHoldTarget(mav_.GetRawPose(), mav_.get_current_yaw());
    publishQrHold(mav_, hold_);
    started_ = ros::SteadyTime::now();
    if (!call_.start(context_->finish_client, service)) return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus QrFinish::onRunning() {
    publishQrHold(mav_, hold_);
    if (!mav_.hasFreshNavigationData() || (ros::SteadyTime::now()-started_).toSec() > 5)
        return BT::NodeStatus::FAILURE;
    uav_qr_mission::FinishSurvey::Response response;
    bool transport;
    if (!call_.poll(response, transport)) return BT::NodeStatus::RUNNING;
    if (!transport || !response.accepted || !response.result.survey_finished)
        return BT::NodeStatus::FAILURE;
    for (const auto& target : response.result.targets) {
        if (target.content != drop_ || !target.position_valid || !target.revision) continue;
        const auto& p = target.position;
        if (!std::isfinite(p.latitude) || !std::isfinite(p.longitude) ||
            std::abs(p.latitude) > 90 || std::abs(p.longitude) > 180 || p.header.stamp.isZero())
            return BT::NodeStatus::FAILURE;
        setOutput("latitude", p.latitude);
        setOutput("longitude", p.longitude);
        ROS_INFO("QrFinish: full grid completed; frozen drop target %s at %.9f, %.9f; all_localized=%d",
                 drop_.c_str(), p.latitude, p.longitude, response.result.all_localized);
        return BT::NodeStatus::SUCCESS;
    }
    ROS_ERROR("QrFinish: drop code has no valid measured position; holding instead of navigating.");
    return BT::NodeStatus::FAILURE;
}
} // namespace maxt
