#include <maxt_pkg/mavkit/maxt_mav.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace maxt {

void MavKit::heartbeatTimerCallback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    auto publish_hold = [&]() {
        auto hold = current_pose_;
        if (has_default_yaw_) {
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, default_yaw_);
            hold.pose.orientation.x = q.x();
            hold.pose.orientation.y = q.y();
            hold.pose.orientation.z = q.z();
            hold.pose.orientation.w = q.w();
        }
        hold.header.stamp = ros::Time::now();
        setpoint_pub_.publish(hold);
    };
    // 如果处于心跳模式，发布当前位置作为设置点
    if (setpoint_mode_ == SetpointMode::HEARTBEAT )
    {
        publish_hold();
    }else if(setpoint_mode_ == SetpointMode::CONTROL){
        // 发布目标位置 (world -> SLAM)
        geometry_msgs::PoseStamped control_pose = target_pose_;
        calib_.transPose(control_pose);
        control_pose.header.stamp = ros::Time::now();
        setpoint_pub_.publish(control_pose);
    }else if(setpoint_mode_ == SetpointMode::RAW_CTRL){
        // 原始目标直接发布，不做 Home 偏移或标定转换。
        mavros_msgs::PositionTarget raw = raw_target_;
        raw.header.stamp = ros::Time::now();
        raw_setpoint_pub_.publish(raw);
    }else if(setpoint_mode_ == SetpointMode::GLOBAL_RAW_CTRL){
        // 全球目标不经过局部 Home 偏移或标定旋转。
        auto target = global_target_;
        target.header.stamp = ros::Time::now();
        global_raw_setpoint_pub_.publish(target);
    }else if(setpoint_mode_ == SetpointMode::CURVE_CTRL){
        loadCurveRaw(ros::Time::now());
        mavros_msgs::PositionTarget raw = curve_raw_target_;
        calib_.transRaw(raw);
        raw_setpoint_pub_.publish(raw);
    }else if(setpoint_mode_ == SetpointMode::SLOW_DOWN){
        // 减速完成及接管逻辑仅在 SLOW_DOWN 中执行。
        const ros::Time now = ros::Time::now();
        loadCurveRaw(now);
        const auto& v = current_twist_.twist.linear;
        const double speed = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
        const double age = (now - current_twist_.header.stamp).toSec();
        const bool fresh = !current_twist_.header.stamp.isZero() && age >= 0.0 && age <= 0.5;
        const bool curve_finished = (now - curve_start_).toSec() >= curve_time_;
        if (curve_finished && fresh && speed <= stop_velocity_tolerance_) {
            if (stop_stable_since_.isZero()) stop_stable_since_ = now;
            if ((now - stop_stable_since_).toSec() >= stop_settle_time_) {
                velocity_transition_ = false;
                setpoint_mode_ = SetpointMode::HEARTBEAT;
                ROS_INFO("[Stop] COMPLETE: measured speed=%.3f m/s", speed);
                publish_hold();
                return;
            }
        } else {
            stop_stable_since_ = ros::Time();
        }
        // 曲线结束后持续保持固定终点，直到实测速度稳定。
        mavros_msgs::PositionTarget raw = curve_raw_target_;
        calib_.transRaw(raw);
        raw_setpoint_pub_.publish(raw);
    }else if(setpoint_mode_ == SetpointMode::STANDBY){
        // 不发布
    }else{
        // 未定义状态
        ROS_ERROR("undefined setpoint mode");
        return;
    }
}

} // namespace maxt
