#include <maxt_pkg/qr_ugv_dispatch.hpp>
#include <cmath>

namespace maxt {

QrUgvDispatch::QrUgvDispatch(ros::NodeHandle nh) {
    bool enabled;
    nh.param("ugv_dispatch/enabled", enabled, true);
    if (!enabled) return;
    nh.param("ugv_dispatch/state_max_age", state_max_age_, state_max_age_);
    intent_ = nh.advertise<slicer_air_ground_msgs::V2Command>("/air_ground/v2/uav_intent", 8, false);
    state_ = nh.subscribe("/air_ground/v2/state", 1, &QrUgvDispatch::onUgvState, this);
}

void QrUgvDispatch::begin(const std::string& mission, const std::string& target,
                          const std::vector<std::string>& ids) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mission_ == mission) return;
    mission_ = mission;
    target_ = target;
    ids_ = std::set<std::string>(ids.begin(), ids.end());
    submitted_.clear();
}

void QrUgvDispatch::onUgvState(const slicer_air_ground_msgs::V2State::ConstPtr& message) {
    const bool ready = message->version == 2 && message->vehicle_id == "ugv" &&
        message->reason != "MONITOR_ONLY";
    std::lock_guard<std::mutex> lock(mutex_);
    state_stamp_ = ready ? message->updated_at : 0.0;
    state_received_ = ros::SteadyTime::now();
}

void QrUgvDispatch::observe(const uav_qr_mission::MissionResult& result) {
    if (!intent_) return;
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock() || mission_.empty() || result.mission_id != mission_ ||
        result.position_target_id != target_) return;

    const auto now = ros::Time::now();
    if (result.header.stamp.isZero() || result.header.stamp > now ||
        (now - result.header.stamp).toSec() > 2.0) return;

    for (std::size_t i = 0; i < result.targets.size(); ++i) {
        const auto& record = result.targets[i];
        const bool is_target = record.content == target_;
        if (!ids_.count(record.content) || submitted_.count(record.content) ||
            is_target != record.is_drop_target || !record.found ||
            !record.position_valid || record.revision == 0) continue;

        const auto& p = record.position;
        if (p.header.frame_id != "wgs84" || p.header.stamp.isZero() || p.header.stamp > now ||
            !std::isfinite(p.latitude) || !std::isfinite(p.longitude) ||
            std::abs(p.latitude) > 90 || std::abs(p.longitude) > 180) {
            ROS_WARN_THROTTLE(5.0, "[AG_Client] 二维码缺少有效的地面 WGS84 坐标或测量时间");
            continue;
        }

        const double age = now.toSec() - state_stamp_;
        if (intent_.getNumSubscribers() == 0 || state_received_.isZero() ||
            !std::isfinite(state_stamp_) || state_stamp_ <= 0.0 || age < 0.0 ||
            age > state_max_age_ ||
            (ros::SteadyTime::now() - state_received_).toSec() > state_max_age_) {
            ROS_WARN_THROTTLE(5.0, "[AG_Client] 等待 V2 gateway 和新鲜的 UGV 状态");
            return;
        }

        slicer_air_ground_msgs::V2Command message;
        const char* op = is_target ? "SET_TARGET" : "REPORT_QR";
        message.op = op;
        message.task_id = mission_;
        if (result.target_images.size() == result.targets.size()) message.image = result.target_images[i];
        const auto image_id = message.image.data.empty() ? std::string() : record.content;
        if (is_target) {
            message.has_target = true;
            message.target.lat = p.latitude;
            message.target.lon = p.longitude;
            message.target.image_id = image_id;
        } else {
            message.qr.lat = p.latitude;
            message.qr.lon = p.longitude;
            message.qr.qr_id = record.content;
            message.qr.image_id = image_id;
            message.qr.measured_at = p.header.stamp.toSec();
            message.qr.revision = record.revision;
            message.qr.has_altitude = std::isfinite(p.altitude);
            if (message.qr.has_altitude) message.qr.altitude_m = p.altitude;
        }
        intent_.publish(message);
        submitted_.insert(record.content);
        ROS_INFO("[AG_Client] 已向 V2 gateway 提交 %s [%s] %.9f, %.9f (task_id=%s)",
                 op, record.content.c_str(), p.latitude, p.longitude, mission_.c_str());
    }
}

}  // namespace maxt
