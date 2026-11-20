#include <maxt_pkg/maxt_nodes/qr_survey_setup_node.hpp>
#include <set>

namespace maxt {
QrSurveySetup::QrSurveySetup(const std::string& name, const BT::NodeConfiguration& config,
                           ros::NodeHandle nh, std::shared_ptr<QrSurveyContext> context)
    : BT::StatefulActionNode(name, config), params_(nh, "qr_survey"), context_(context) {}

BT::PortsList QrSurveySetup::providedPorts() {
    return {BT::OutputPort<std::string>("mission_id"), BT::OutputPort<std::string>("drop_target_id"),
            BT::OutputPort<RegionWaypoints>("waypoints"), BT::OutputPort<int>("waypoint_count"),
            BT::OutputPort<double>("survey_height"), BT::OutputPort<double>("drop_height")};
}

BT::NodeStatus QrSurveySetup::onStart() {
    double corner_x, corner_y, step;
    if (params_.hasParam("corner_x") || params_.hasParam("corner_y")) {
        if (!params_.getParam("corner_x", corner_x) || !params_.getParam("corner_y", corner_y)) {
            ROS_ERROR("QrSurveySetup: configure both corner_x and corner_y.");
            return BT::NodeStatus::FAILURE;
        }
    } else {
        params_.param("length_x", corner_x, 20.0);
        params_.param("length_y", corner_y, 20.0);
        ROS_WARN("QrSurveySetup: legacy length_x/length_y; migrate to signed corner_x/corner_y.");
    }
    params_.param("step", step, 10.0);
    params_.param("survey_height", survey_height_, 5.0);
    params_.param("drop_height", drop_height_, 5.0);
    auto& request = service_.request;
    std::vector<std::string> reports;
    if (!params_.getParam("report_id", reports))
        params_.getParam("ordinary_ids", reports);
    if (!params_.getParam("target_id", request.position_target_id))
        params_.param<std::string>("drop_target_id", request.position_target_id, "QR001");
    std::set<std::string> unique(reports.begin(), reports.end());
    unique.insert(request.position_target_id);
    if (reports.size() != 2 || unique.size() != 3 || unique.count("") ||
        !std::isfinite(survey_height_) || survey_height_ <= 0 ||
        !std::isfinite(drop_height_) || drop_height_ <= 0) {
        ROS_ERROR("QrSurveySetup: report_id needs two distinct codes, target_id a third, and heights must be positive.");
        return BT::NodeStatus::FAILURE;
    }
    try {
        points_ = stopSurveyGrid(corner_x, corner_y, step);
        ROS_INFO("QrSurveySetup: takeoff-heading rectangle (0,0) to (%.2f,%.2f), step=%.2f; X forward, Y left.",
                 corner_x, corner_y, step);
    } catch (const std::exception& e) {
        ROS_ERROR("QrSurveySetup: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }
    request.target_ids = reports;
    request.target_ids.push_back(request.position_target_id);
    request.mission_id = "qr_survey_"+std::to_string(ros::WallTime::now().toNSec());
    request.capture_only = true;
    accepted_ = false;
    started_ = retry_at_ = ros::SteadyTime::now();
    if (!call_.start(context_->task_client, service_)) return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus QrSurveySetup::onRunning() {
    const auto now = ros::SteadyTime::now();
    if ((now-started_).toSec() > 15) {
        ROS_ERROR("QrSurveySetup: registration/RTK/IMU readiness timed out before takeoff.");
        return BT::NodeStatus::FAILURE;
    }
    if (!accepted_) {
        uav_qr_mission::SetTask::Response response;
        bool transport;
        if (!call_.poll(response, transport)) return BT::NodeStatus::RUNNING;
        if (!transport) {
            if (now >= retry_at_) {
                call_.start(context_->task_client, service_);
                retry_at_ = now+ros::WallDuration(0.5);
            }
            return BT::NodeStatus::RUNNING;
        }
        if (!response.accepted) {
            ROS_ERROR("QrSurveySetup rejected: %s", response.status.c_str());
            return BT::NodeStatus::FAILURE;
        }
        accepted_ = true;
        context_->beginUgvMission(service_.request);
    }
    uav_qr_mission::MissionResult result;
    if (!context_->snapshot(service_.request.mission_id, result)) {
        ROS_WARN_THROTTLE(2.0, "QrSurveySetup: waiting for mission heartbeat/registration.");
        return BT::NodeStatus::RUNNING;
    }
    if (result.geolocation_input_status != "VALID") {
        ROS_WARN_THROTTLE(2.0, "QrSurveySetup: waiting for geolocation: %s",
                          result.geolocation_input_status.c_str());
        return BT::NodeStatus::RUNNING;
    }
    setOutput("mission_id", service_.request.mission_id);
    setOutput("drop_target_id", service_.request.position_target_id);
    setOutput("waypoints", points_);
    setOutput("waypoint_count", static_cast<int>(points_.size()));
    setOutput("survey_height", survey_height_);
    setOutput("drop_height", drop_height_);
    ROS_INFO("QrSurveySetup: %zu stationary stops registered as %s", points_.size(),
             service_.request.mission_id.c_str());
    return BT::NodeStatus::SUCCESS;
}
} // namespace maxt
