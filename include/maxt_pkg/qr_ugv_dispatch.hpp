#pragma once

#include <ag_client/client.hpp>
#include <uav_qr_mission/MissionResult.h>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

namespace maxt {

// The QR mission owns geolocation. This adapter only forwards the selected
// target's first valid ground WGS84 point, never an aircraft position or height.
class QrUgvDispatch {
    std::unique_ptr<ag_client::Client> client_;
    std::mutex mutex_;
    std::string mission_, target_;
    bool forwarded_{false};

public:
    explicit QrUgvDispatch(ros::NodeHandle nh) {
        bool enabled;
        nh.param("ugv_dispatch/enabled", enabled, true);
        if (!enabled) {
            ROS_INFO("[AG_Client] QR target dispatch disabled by ugv_dispatch/enabled");
            return;
        }
        ag_client::Options options;
        nh.param("ugv_dispatch/state_timeout", options.state_timeout, options.state_timeout);
        nh.param("ugv_dispatch/state_max_age", options.state_max_age, options.state_max_age);
        nh.param("ugv_dispatch/execution_max_age", options.execution_max_age, options.execution_max_age);
        nh.param("ugv_dispatch/connect_timeout", options.connect_timeout, options.connect_timeout);
        nh.param("ugv_dispatch/receipt_timeout", options.receipt_timeout, options.receipt_timeout);
        // Absolute formal topics by default. ROS remapping supports isolated tests.
        client_.reset(new ag_client::Client(options));
    }

    // Called once after the existing QR set_task service accepts this mission.
    // No ROS call, waiting, flight control or join takes place on this BT path.
    void begin(const std::string& mission, const std::string& target) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mission_ == mission && target_ == target) return;
        mission_ = mission;
        target_ = target;
        forwarded_ = false;
    }

    void observe(const uav_qr_mission::MissionResult& result) {
        if (!client_) return;
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock() || forwarded_ || mission_.empty() || result.mission_id != mission_ ||
            result.position_target_id != target_) return;
        const auto now = ros::Time::now();
        if (result.header.stamp.isZero() || result.header.stamp > now ||
            (now-result.header.stamp).toSec() > 2.0) return;
        for (const auto& target : result.targets) {
            if (target.content != target_ || !target.is_drop_target || !target.found ||
                !target.position_valid || target.revision == 0) continue;
            const auto& p = target.position;
            if (p.header.frame_id != "wgs84" || p.header.stamp.isZero() || p.header.stamp > now ||
                !std::isfinite(p.latitude) || !std::isfinite(p.longitude) ||
                std::abs(p.latitude) > 90 || std::abs(p.longitude) > 180) {
                ROS_WARN_THROTTLE(5.0, "[AG_Client] 未发送 QR 目标：缺少有效地面 WGS84 坐标/测量时间");
                return;
            }
            ag_client::Target ground;
            ground.mission_id = mission_;
            ground.label = target_;
            ground.latitude = p.latitude;
            ground.longitude = p.longitude;
            std::string reason;
            forwarded_ = client_->submitTarget(ground, &reason);
            if (!forwarded_)
                ROS_WARN_STREAM_THROTTLE(5.0, "[AG_Client] QR 目标暂未入队：" << reason);
            return;
        }
    }
};

}  // namespace maxt
