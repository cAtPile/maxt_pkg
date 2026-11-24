#include <maxt_pkg/maxt_nodes/qr_capture_node.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace maxt {
QrCapture::QrCapture(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav,
                     ros::NodeHandle nh, std::shared_ptr<QrSurveyContext> context)
    : MavActionNode(name, config, mav), params_(nh, "qr_survey"), context_(context) {}

BT::PortsList QrCapture::providedPorts() {
    return {BT::InputPort<std::string>("mission_id"),
            BT::InputPort<int>("waypoint_number")};
}

BT::NodeStatus QrCapture::onStart() {
    if (!mav_.hasFreshNavigationData() || !getInput("mission_id", service_.request.mission_id) ||
        !getInput("waypoint_number", waypoint_))
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
    attempts_ = 0;
    phase_ = Phase::Settling;
    return BT::NodeStatus::RUNNING;
}

bool QrCapture::stable() {
    const auto pose = mav_.GetRawPose();
    const auto twist = mav_.getCurrentTwist();
    const auto& p = pose.pose.position;
    const auto& v = twist.twist.linear;
    const auto& q = pose.pose.orientation;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    quaternion.normalize();
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    distance_ = std::sqrt(std::pow(p.x-hold_.position.x, 2) +
        std::pow(p.y-hold_.position.y, 2) + std::pow(p.z-hold_.position.z, 2));
    velocity_ = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    roll_deg_ = roll * 180 / M_PI;
    pitch_deg_ = pitch * 180 / M_PI;
    return distance_ <= tolerance_ && velocity_ <= speed_ &&
        std::abs(roll_deg_) <= tilt_ && std::abs(pitch_deg_) <= tilt_;
}

bool QrCapture::cancel() {
    auto request = service_;
    request.request.command = uav_qr_mission::CaptureQrRequest::CANCEL;
    return cancel_call_.start(context_->capture_client, request);
}

BT::NodeStatus QrCapture::skip(const char* reason) const {
    ROS_WARN("QrCapture: waypoint %d SKIPPED (%s), attempts=%d; continuing survey.",
             waypoint_, reason, attempts_);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus QrCapture::retryOrSkip(const char* reason) {
    if (attempts_ >= 2 || (ros::SteadyTime::now()-started_).toSec() + settle_ + stable_time_ >= settle_timeout_)
        return skip(reason);
    phase_ = Phase::Settling;
    stable_since_ = ros::SteadyTime();
    ROS_WARN("QrCapture: waypoint %d will retry (%s).", waypoint_, reason);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus QrCapture::onRunning() {
    using Request = uav_qr_mission::CaptureQrRequest;
    if (!mav_.hasFreshNavigationData() || !mav_.isArmed() || mav_.getMode() != "OFFBOARD") {
        if (phase_ != Phase::Settling && phase_ != Phase::Cancelling) cancel();
        ROS_ERROR("QrCapture: waypoint %d stopped: stale navigation, disarmed or OFFBOARD exited.", waypoint_);
        return BT::NodeStatus::FAILURE;
    }
    publishQrHold(mav_, hold_);
    const auto now = ros::SteadyTime::now();
    uav_qr_mission::MissionResult result;
    if (!context_->snapshot(service_.request.mission_id, result)) {
        if (phase_ != Phase::Settling && phase_ != Phase::Cancelling) cancel();
        ROS_ERROR("QrCapture: mission heartbeat lost or mission replaced.");
        return BT::NodeStatus::FAILURE;
    }

    const bool is_stable = stable();
    const bool matching = result.capture_id == service_.request.capture_id;
    const bool terminal = matching && result.capture_done;
    const double point_elapsed = (now-started_).toSec();
    const double phase_elapsed = (now-capture_started_).toSec();
    Request& request = service_.request;
    if (phase_ == Phase::Settling) {
        if (point_elapsed >= settle_timeout_) {
            ROS_WARN("QrCapture: waypoint %d settle timeout: speed=%.3f m/s roll=%.1f deg pitch=%.1f deg offset=%.3f m.",
                     waypoint_, velocity_, roll_deg_, pitch_deg_, distance_);
            return skip("settle timeout");
        }
        if (!is_stable) stable_since_ = ros::SteadyTime();
        else if (stable_since_.isZero()) stable_since_ = now;
        if (stable_since_.isZero() || (now-stable_since_).toSec() < stable_time_+settle_)
            return BT::NodeStatus::RUNNING;
        request.capture_id = request.mission_id + "/" + std::to_string(now.toNSec());
        request.command = Request::START_PROVISIONAL;
        request.not_before = ros::Time::now();
        request.timeout_s = std::min(capture_timeout_, settle_timeout_ - point_elapsed);
        if (!call_.start(context_->capture_client, service_)) {
            ROS_ERROR("QrCapture: previous capture RPC still in flight.");
            return BT::NodeStatus::FAILURE;
        }
        capture_started_ = now;
        phase_ = Phase::Starting;
        ++attempts_;
        ROS_INFO("QrCapture: waypoint %d attempt %d: stationary exposure requested: %s",
                 waypoint_, attempts_, request.capture_id.c_str());
        return BT::NodeStatus::RUNNING;
    }

    uav_qr_mission::CaptureQr::Response response;
    bool transport = false;
    if (phase_ == Phase::Cancelling) {
        if (cancel_call_.poll(response, transport) && transport && response.accepted) {
            // Wait for the START worker too, before reusing it for another attempt.
            if (call_.poll(response, transport)) return retryOrSkip("capture cancelled");
        }
        if (phase_elapsed <= 1.0) return BT::NodeStatus::RUNNING;
        ROS_ERROR("QrCapture: waypoint %d cancellation or prior request could not be verified.", waypoint_);
        return BT::NodeStatus::FAILURE;
    }

    if (phase_ == Phase::Confirming) {
        if (terminal && call_.poll(response, transport)) {
            if (result.capture_success) return BT::NodeStatus::SUCCESS;
            ROS_WARN("QrCapture: waypoint %d capture result: %s.", waypoint_, result.capture_status.c_str());
            return retryOrSkip(result.capture_status.c_str());
        }
        if (phase_elapsed <= request.timeout_s + 1.0) return BT::NodeStatus::RUNNING;
        ROS_ERROR("QrCapture: waypoint %d confirmed capture result unavailable.", waypoint_);
        cancel();
        return BT::NodeStatus::FAILURE;
    }

    if (!is_stable || phase_elapsed > request.timeout_s + 1.0 || point_elapsed >= settle_timeout_) {
        ROS_WARN("QrCapture: waypoint %d attempt %d %s: speed=%.3f m/s roll=%.1f deg pitch=%.1f deg offset=%.3f m.",
                 waypoint_, attempts_, is_stable ? "frame wait timeout" : "motion during capture",
                 velocity_, roll_deg_, pitch_deg_, distance_);
        if (!cancel()) return BT::NodeStatus::FAILURE;
        phase_ = Phase::Cancelling;
        capture_started_ = now;
        return BT::NodeStatus::RUNNING;
    }

    if (phase_ == Phase::Starting) {
        if (!call_.poll(response, transport)) return BT::NodeStatus::RUNNING;
        if (!transport || !response.accepted) {
            if (transport && response.status == "REQUEST_EXPIRED") return retryOrSkip("request expired");
            ROS_ERROR("QrCapture: START rejected at waypoint %d: %s.", waypoint_, response.status.c_str());
            cancel();
            return BT::NodeStatus::FAILURE;
        }
        phase_ = Phase::WaitingFrame;
    }

    if (terminal) return retryOrSkip(result.capture_status.c_str());
    if (matching && result.capture_status == "FRAME_READY") {
        request.command = Request::CONFIRM;
        if (!call_.start(context_->capture_client, service_)) return BT::NodeStatus::FAILURE;
        phase_ = Phase::Confirming;
        capture_started_ = now;
    }
    return BT::NodeStatus::RUNNING;
}

void QrCapture::onHalted() {
    if (phase_ != Phase::Settling && phase_ != Phase::Cancelling) cancel();
    mav_.setSetpointMode(SetpointMode::SLOW_DOWN);
}
} // namespace maxt
