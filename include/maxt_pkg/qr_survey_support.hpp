#pragma once
#include <maxt_pkg/maxt_nodes/region_waypoints.hpp>
#include <maxt_pkg/mavkit/maxt_mav.hpp>
#include <uav_qr_mission/MissionResult.h>
#include <uav_qr_mission/SetTask.h>
#include <uav_qr_mission/CaptureQr.h>
#include <uav_qr_mission/FinishSurvey.h>
#include <mutex>
#include <thread>
#include <memory>
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace maxt {

// Endpoints are included, unlike the continuous strip-coverage generator.
// 20 x 20 / step 10 => nine stops, with adjacent rows connected serpentine.
inline RegionWaypoints stopSurveyGrid(double length_x, double length_y, double step) {
    if (!std::isfinite(length_x) || !std::isfinite(length_y) || !std::isfinite(step) ||
        length_x <= 0 || length_y <= 0 || step <= 0)
        throw std::invalid_argument("Survey dimensions and step must be finite and positive");
    const double nx = std::ceil(length_x/step), ny = std::ceil(length_y/step);
    if ((nx+1)*(ny+1) > 10000) throw std::invalid_argument("Survey exceeds 10000 stops");
    RegionWaypoints points;
    for (int x = 0; x <= static_cast<int>(nx); ++x) {
        for (int j = 0; j <= static_cast<int>(ny); ++j) {
            const int y = x%2 == 0 ? j : static_cast<int>(ny)-j;
            points.push_back({{std::min(x*step, length_x), std::min(y*step, length_y)}});
        }
    }
    return points;
}

// A ROS service can block indefinitely over a radio. Never call it on a BT tick
// or join a blocked worker on halt. At most one outstanding call per instance;
// its shared state owns all memory until the call eventually returns.
template<class Service> class AsyncQrService {
    struct State {
        std::mutex mutex;
        bool done{false}, ok{false};
        Service service;
    };
    std::shared_ptr<State> state_;
public:
    bool start(ros::ServiceClient client, const Service& service) {
        if (state_) {
            std::lock_guard<std::mutex> lock(state_->mutex);
            if (!state_->done) return false;
        }
        auto state = std::make_shared<State>();
        state->service = service;
        state_ = state;
        std::thread([state, client]() mutable {
            bool ok = false;
            try { ok = client.call(state->service); }
            catch (const std::exception& e) { ROS_ERROR("QR service transport: %s", e.what()); }
            std::lock_guard<std::mutex> lock(state->mutex);
            state->ok = ok;
            state->done = true;
        }).detach();
        return true;
    }
    bool poll(typename Service::Response& response, bool& ok) const {
        if (!state_) return false;
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (!state_->done) return false;
        response = state_->service.response;
        ok = state_->ok;
        return true;
    }
};

class QrSurveyContext {
    std::mutex mutex_;
    uav_qr_mission::MissionResult latest_;
    ros::SteadyTime received_;
    ros::Subscriber subscriber_;
public:
    ros::ServiceClient task_client, capture_client, finish_client;
    explicit QrSurveyContext(ros::NodeHandle nh) {
        std::string ns;
        nh.param<std::string>("qr_survey/mission_namespace", ns, "/uav_qr_mission");
        task_client = nh.serviceClient<uav_qr_mission::SetTask>(ns+"/set_task");
        capture_client = nh.serviceClient<uav_qr_mission::CaptureQr>(ns+"/capture");
        finish_client = nh.serviceClient<uav_qr_mission::FinishSurvey>(ns+"/finish_survey");
        subscriber_ = nh.subscribe<uav_qr_mission::MissionResult>(ns+"/result", 1,
            [this](const uav_qr_mission::MissionResult::ConstPtr& message) {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_ = *message;
                received_ = ros::SteadyTime::now();
            });
    }
    bool snapshot(const std::string& mission, uav_qr_mission::MissionResult& result) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (received_.isZero() || (ros::SteadyTime::now()-received_).toSec() > 2.0 ||
            latest_.mission_id != mission) return false;
        result = latest_;
        return true;
    }
};

// RAW_CTRL publishes raw_target_, not target_pose_. Build an actual fixed
// position target, so every waiting phase continues sending the same setpoint.
inline mavros_msgs::PositionTarget qrHoldTarget(const geometry_msgs::PoseStamped& pose, double yaw) {
    using T = mavros_msgs::PositionTarget;
    T target;
    target.coordinate_frame = T::FRAME_LOCAL_NED;  // MAVROS ROS-side values are ENU.
    target.type_mask = T::IGNORE_VX | T::IGNORE_VY | T::IGNORE_VZ |
        T::IGNORE_AFX | T::IGNORE_AFY | T::IGNORE_AFZ | T::IGNORE_YAW_RATE;
    target.position = pose.pose.position;
    target.yaw = yaw;
    return target;
}

inline void publishQrHold(MavKit& mav, const mavros_msgs::PositionTarget& target) {
    mav.setRawTarget(target, false);
    mav.setSetpointMode(SetpointMode::RAW_CTRL);
}
} // namespace maxt
