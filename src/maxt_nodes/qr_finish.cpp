#include <maxt_pkg/maxt_nodes/qr_finish_node.hpp>
namespace maxt {
BT::PortsList QrFinish::providedPorts() {
    return {BT::InputPort<std::string>("mission_id"), BT::InputPort<std::string>("drop_target_id"),
            BT::OutputPort<double>("latitude"), BT::OutputPort<double>("longitude")};
}
BT::NodeStatus QrFinish::onStart() {
    uav_qr_mission::FinishSurvey service;
    if (!getInput("mission_id", service.request.mission_id) ||
        !getInput("drop_target_id", drop_) || !mav_.hasFreshNavigationData()) {
        ROS_ERROR("QrFinish: navigation blocked: missing mission/drop input or stale navigation data.");
        return BT::NodeStatus::FAILURE;
    }
    hold_ = qrHoldTarget(mav_.GetRawPose(), mav_.get_current_yaw());
    publishQrHold(mav_, hold_);
    started_ = ros::SteadyTime::now();
    if (!call_.start(context_->finish_client, service)) {
        ROS_ERROR("QrFinish: navigation blocked: could not start finish_survey service call.");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus QrFinish::onRunning() {
    publishQrHold(mav_, hold_);
    if (!mav_.hasFreshNavigationData() || (ros::SteadyTime::now()-started_).toSec() > 5) {
        ROS_ERROR("QrFinish: navigation blocked: stale navigation data or finish_survey timeout (5s).");
        return BT::NodeStatus::FAILURE;
    }
    uav_qr_mission::FinishSurvey::Response response;
    bool transport;
    if (!call_.poll(response, transport)) return BT::NodeStatus::RUNNING;
    if (!transport || !response.accepted || !response.result.survey_finished) {
        ROS_ERROR("QrFinish: navigation blocked: transport=%d accepted=%d finished=%d status=%s",
                  transport, response.accepted, response.result.survey_finished, response.status.c_str());
        return BT::NodeStatus::FAILURE;
    }
    for (const auto& target : response.result.targets) {
        ROS_INFO("QrFinish: target=%s seen_in_survey=%d position_valid=%d revision=%u reason=%s",
                 target.content.c_str(), target.found, target.position_valid,
                 target.revision, target.status.c_str());
    }
    for (const auto& target : response.result.targets) {
        if (target.content != drop_ || !target.position_valid || !target.revision) continue;
        const auto& p = target.position;
        if (!std::isfinite(p.latitude) || !std::isfinite(p.longitude) ||
            std::abs(p.latitude) > 90 || std::abs(p.longitude) > 180 || p.header.stamp.isZero()) {
            ROS_ERROR("QrFinish: navigation blocked: target=%s has invalid coordinates or zero measurement stamp.",
                      drop_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        setOutput("latitude", p.latitude);
        setOutput("longitude", p.longitude);
        ROS_INFO("QrFinish: full grid completed; frozen drop target %s at %.9f, %.9f; all_localized=%d",
                 drop_.c_str(), p.latitude, p.longitude, response.result.all_localized);
        return BT::NodeStatus::SUCCESS;
    }
    ROS_ERROR("QrFinish: navigation blocked: target=%s has no valid measured position; "
              "geolocation_input=%s; entering SurveyFailureHold. See target seen/position/reason above.",
              drop_.c_str(), response.result.geolocation_input_status.c_str());
    return BT::NodeStatus::FAILURE;
}
} // namespace maxt
