#include <maxt_pkg/mavkit/maxt_mav.hpp>
#include <tf2/LinearMath/Quaternion.h>

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
    }else if(setpoint_mode_ == SetpointMode::CURVE_CTRL){
        loadCurveRaw(ros::Time::now());
        mavros_msgs::PositionTarget raw = curve_raw_target_;
        calib_.transRaw(raw);
        raw_setpoint_pub_.publish(raw);
    }else if(setpoint_mode_ == SetpointMode::SLOW_DOWN){
        // 减速完成及接管逻辑仅在 SLOW_DOWN 中执行。
        const ros::Time now = ros::Time::now();
        loadCurveRaw(now);
        // 必须同时完成加速度归零；仅速度过零就退出会截断仍在执行的曲线。
        const bool transition_finished = velocity_transition_ &&
            (now - curve_start_).toSec() >= curve_time_;
        if (transition_finished &&
            (transition_end_mode_ != SetpointMode::HEARTBEAT ||
             curve_state_.velocity.cwiseAbs().maxCoeff() <= stop_velocity_tolerance_)) {
            velocity_transition_ = false;
            setpoint_mode_ = transition_end_mode_;
            if (transition_end_mode_ == SetpointMode::HEARTBEAT) {
                publish_hold();
            } else {
                auto raw = raw_target_;
                raw.header.stamp = now;
                raw_setpoint_pub_.publish(raw);
            }
            return;
        }
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
