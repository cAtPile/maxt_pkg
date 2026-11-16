#include <maxt_pkg/maxt_nodes/qr_capture_node.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace maxt {
QrCapture::QrCapture(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav,
                     ros::NodeHandle nh, std::shared_ptr<QrSurveyContext> context)
    : MavActionNode(name, config, mav), params_(nh, "qr_survey"), context_(context) {}

BT::PortsList QrCapture::providedPorts() {
    return {BT::InputPort<std::string>("mission_id")};
}

BT::NodeStatus QrCapture::onStart() {
    if (!mav_.hasFreshNavigationData() || !getInput("mission_id", service_.request.mission_id))
        return BT::NodeStatus::FAILURE;
    params_.param("settle_seconds", settle_, 1.0);
    params_.param("stable_seconds", stable_time_, 0.4);
    params_.param("max_capture_speed", speed_, 0.2);
    params_.param("max_capture_tilt_deg", tilt_, 10.0);
    params_.param("hold_tolerance", tolerance_, 0.2);
    params_.param("settle_timeout", settle_timeout_, 20.0);
    params_.param("capture_timeout", capture_timeout_, 3.0);
    for (double v : {settle_, stable_time_, speed_, tilt_, tolerance_, settle_timeout_, capture_timeout_}) {
        if (!std::isfinite(v) || v <= 0) return BT::NodeStatus::FAILURE;
    }
    if (settle_ < 1 || tilt_ >= 90 || capture_timeout_ > 10 ||
        settle_timeout_ <= settle_+stable_time_) return BT::NodeStatus::FAILURE;
    hold_ = qrHoldTarget(mav_.GetRawPose(), mav_.get_current_yaw());
    publishQrHold(mav_, hold_);
    started_ = ros::SteadyTime::now();
    stable_since_ = ros::SteadyTime();
    requested_ = accepted_ = false;
    service_.request.capture_id = service_.request.mission_id+"/"+std::to_string(started_.toNSec());
    return BT::NodeStatus::RUNNING;
}

bool QrCapture::stable() const {
    const auto pose = mav_.GetRawPose();
    const auto twist = mav_.getCurrentTwist();
    const auto& p = pose.pose.position;
    const auto& v = twist.twist.linear;
    const auto& q = pose.pose.orientation;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    quaternion.normalize();
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    const double distance = std::sqrt(std::pow(p.x-hold_.position.x, 2)+
        std::pow(p.y-hold_.position.y, 2)+std::pow(p.z-hold_.position.z, 2));
    return distance <= tolerance_ && std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z) <= speed_ &&
        std::abs(roll) <= tilt_*M_PI/180 && std::abs(pitch) <= tilt_*M_PI/180;
}

void QrCapture::cancel() {
    if (!requested_) return;
    auto request = service_;
    request.request.command = uav_qr_mission::CaptureQrRequest::CANCEL;
    cancel_call_.start(context_->capture_client, request);
}

BT::NodeStatus QrCapture::onRunning() {
    if (!mav_.hasFreshNavigationData() || !mav_.isArmed() || mav_.getMode() != "OFFBOARD") {
        cancel();
        return BT::NodeStatus::FAILURE;
    }
    publishQrHold(mav_, hold_);
    const auto now = ros::SteadyTime::now();
    uav_qr_mission::MissionResult result;
    if (!context_->snapshot(service_.request.mission_id, result)) {
        cancel();
        ROS_ERROR("QrCapture: mission heartbeat lost or mission replaced.");
        return BT::NodeStatus::FAILURE;
    }
    const bool is_stable = stable();
    if (!requested_) {
        if ((now-started_).toSec() > settle_timeout_) return BT::NodeStatus::FAILURE;
        if (!is_stable) stable_since_ = ros::SteadyTime();
        else if (stable_since_.isZero()) stable_since_ = now;
        if (stable_since_.isZero() || (now-stable_since_).toSec() < stable_time_+settle_)
            return BT::NodeStatus::RUNNING;
        // A fresh image/depth pair must be acquired AFTER this point in time.
        service_.request.command = uav_qr_mission::CaptureQrRequest::START;
        service_.request.not_before = ros::Time::now();
        service_.request.timeout_s = capture_timeout_;
        if (!call_.start(context_->capture_client, service_)) return BT::NodeStatus::FAILURE;
        capture_started_ = now;
        requested_ = true;
        ROS_INFO("QrCapture: stationary exposure requested: %s", service_.request.capture_id.c_str());
        return BT::NodeStatus::RUNNING;
    }
    if (!is_stable || (now-capture_started_).toSec() > capture_timeout_+1.0) {
        cancel();
        ROS_ERROR("QrCapture: motion during exposure or capture timeout.");
        return BT::NodeStatus::FAILURE;
    }
    if (!accepted_) {
        uav_qr_mission::CaptureQr::Response response;
        bool transport;
        if (!call_.poll(response, transport)) return BT::NodeStatus::RUNNING;
        if (!transport || !response.accepted) {
            cancel();
            ROS_ERROR("QrCapture rejected: %s", response.status.c_str());
            return BT::NodeStatus::FAILURE;
        }
        accepted_ = true;
    }
    if (result.capture_id != service_.request.capture_id || !result.capture_done)
        return BT::NodeStatus::RUNNING;
    return result.capture_success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void QrCapture::onHalted() {
    cancel();
    mav_.setSetpointMode(SetpointMode::SLOW_DOWN);
}
} // namespace maxt
