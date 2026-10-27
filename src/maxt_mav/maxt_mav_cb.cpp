#include <maxt_pkg/maxt_mav.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace maxt {

// --- 回调函数层 (运行在 AsyncSpinner 线程) ---

void MavKit::stateCb(const mavros_msgs::State::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    current_state_ = *msg;
    ROS_DEBUG("MavKit: State updated: %s", current_state_.connected ? "connected" : "disconnected");
}

void MavKit::extendedStateCb(const mavros_msgs::ExtendedState::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    current_extended_state_ = *msg;
}

void MavKit::poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    current_pose_ = *msg;
    has_pose_ = true;
}

void MavKit::twistCb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    current_twist_ = *msg;
}

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
        mavros_msgs::PositionTarget raw = raw_target_;
        calib_.transRaw(raw);
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
                calib_.transRaw(raw);
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


// --- 线程安全的数据查询接口 (运行在 BT 线程) ---

bool MavKit::isConnected() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return current_state_.connected;
}

bool MavKit::isArmed() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return current_state_.armed;
}

bool MavKit::isLand() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return current_extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
}

std::string MavKit::getMode() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return current_state_.mode;
}

geometry_msgs::PoseStamped MavKit::getCurrentPose() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    geometry_msgs::PoseStamped out = current_pose_;
    calib_.untransPose(out);
    return out;
}

geometry_msgs::TwistStamped MavKit::getCurrentTwist() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    geometry_msgs::TwistStamped out = current_twist_;
    calib_.untransTwist(out);
    return out;
}

double MavKit::get_current_yaw() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    tf2::Quaternion q(current_pose_.pose.orientation.x,
                      current_pose_.pose.orientation.y,
                      current_pose_.pose.orientation.z,
                      current_pose_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void MavKit::setDefaultYaw(double yaw) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    default_yaw_ = yaw;
    has_default_yaw_ = true;
    ROS_INFO("MavKit: Default yaw locked to %.4f rad (%.2f deg)",
             yaw, yaw * 180.0 / M_PI);
}

//todo 相对化处理home
void MavKit::setTargetPose(double x, double y, double z) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    target_pose_.header.stamp = ros::Time::now();
    target_pose_.header.frame_id = "map"; // 或者是你的坐标系名称
    target_pose_.pose.position.x = x;
    target_pose_.pose.position.y = y;
    target_pose_.pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, default_yaw_);
    target_pose_.pose.orientation.x = q.x();
    target_pose_.pose.orientation.y = q.y();
    target_pose_.pose.orientation.z = q.z();
    target_pose_.pose.orientation.w = q.w();
}

bool MavKit::isReached(double x, double y, double z, double tolerance) const {
    geometry_msgs::PoseStamped curr = getCurrentPose();  // world coords
    double dx = curr.pose.position.x - x;
    double dy = curr.pose.position.y - y;
    double dz = curr.pose.position.z - z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    return distance < tolerance;
}

} // namespace maxt
