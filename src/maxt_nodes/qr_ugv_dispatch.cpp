#include <maxt_pkg/qr_ugv_dispatch.hpp>
#include <json/json.h>
#include <cmath>
#include <memory>

namespace maxt {

QrUgvDispatch::QrUgvDispatch(ros::NodeHandle nh) {
    bool enabled;
    nh.param("ugv_dispatch/enabled", enabled, true);
    if (!enabled) return;
    nh.param("ugv_dispatch/state_max_age", state_max_age_, state_max_age_);
    intent_ = nh.advertise<std_msgs::String>("/air_ground/v2/uav_intent", 8, false);
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

void QrUgvDispatch::onUgvState(const std_msgs::String::ConstPtr& message) {
    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    Json::Value snapshot;
    const auto& data = message->data;
    const bool ready = reader->parse(data.data(), data.data() + data.size(), &snapshot, nullptr) &&
        snapshot.isObject() && snapshot["version"] == 2 && snapshot["vehicle_id"] == "ugv" &&
        snapshot["reason"] != "MONITOR_ONLY" && snapshot["updated_at"].isNumeric();
    std::lock_guard<std::mutex> lock(mutex_);
    state_stamp_ = ready ? snapshot["updated_at"].asDouble() : 0.0;
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

    for (const auto& record : result.targets) {
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

        Json::Value point;
        point["lat"] = p.latitude;
        point["lon"] = p.longitude;
        Json::Value command;
        const char* op = is_target ? "SET_TARGET" : "REPORT_QR";
        command["op"] = op;
        command["task_id"] = mission_;
        if (is_target) {
            command["target"] = point;
        } else {
            point["qr_id"] = record.content;
            point["measured_at"] = p.header.stamp.toSec();
            point["revision"] = Json::UInt(record.revision);
            if (std::isfinite(p.altitude)) point["altitude_m"] = p.altitude;
            command["qr"] = point;
        }
        Json::StreamWriterBuilder writer;
        writer["indentation"] = "";
        std_msgs::String message;
        message.data = Json::writeString(writer, command);
        intent_.publish(message);
        submitted_.insert(record.content);
        ROS_INFO("[AG_Client] 已向 V2 gateway 提交 %s [%s] %.9f, %.9f (task_id=%s)",
                 op, record.content.c_str(), p.latitude, p.longitude, mission_.c_str());
    }
}

}  // namespace maxt
