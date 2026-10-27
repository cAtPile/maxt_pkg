/**
 * @file maxt_mav.cpp
 * @brief maxt的mav组件实现
 * @date 4-14
 */
#include <maxt_pkg/maxt_mav.hpp>

namespace maxt {

MavKit::MavKit(ros::NodeHandle& nh) : nh_(nh) {
    // 保持轻量化
}

bool MavKit::mavInit(ros::NodeHandle& nh) {
    nh_ = nh;

    // 1. 初始化基础标志位
    has_pose_ = false;
    setpoint_mode_ = SetpointMode::STANDBY;

    // 2. 加载参数（省略）
    double heartbeat_rate;
    nh_.param<double>("heartbeat_rate", heartbeat_rate, 20.0);
    nh_.param<double>("nav/slow_down_time", slow_down_time_, 1.0);
    nh_.param<double>("nav/stop_velocity_tolerance", stop_velocity_tolerance_, 0.01);

    // 3. 初始化 ROS 通信对象
    state_sub_ = nh_.subscribe<mavros_msgs::State>(
        "/mavros/state", 1, &MavKit::stateCb, this);

    extended_state_sub_ = nh_.subscribe<mavros_msgs::ExtendedState>(
        "/mavros/extended_state", 1, &MavKit::extendedStateCb, this);

    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 1, &MavKit::poseCb, this);

    twist_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
        "/mavros/local_position/velocity_local", 1, &MavKit::twistCb, this);

    setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);

    raw_setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 10);

    // 4. 初始化 Service 客户端
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 5. 启动心跳定时器
    heartbeat_timer_ = nh_.createTimer(
        ros::Duration(1.0 / heartbeat_rate), &MavKit::heartbeatTimerCallback, this);

    ROS_INFO("MavKit initialized. Waiting for MAVROS connection...");
    return true;
}

void MavKit::setSetpointMode(SetpointMode mode) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    const ros::Time now = ros::Time::now();
    if (mode == SetpointMode::SLOW_DOWN) {
        // 重复停止请求不重置减速时间。
        if (velocity_transition_ && transition_end_mode_ == SetpointMode::HEARTBEAT) return;
        startVelocityTransition(Eigen::Vector3d::Zero(), now);
        transition_end_mode_ = SetpointMode::HEARTBEAT;
        return;
    }
    if (velocity_transition_ && setpoint_mode_ == SetpointMode::SLOW_DOWN) {
        if (mode == SetpointMode::HEARTBEAT) {
            // Guidance 初始化/等待反馈也会请求心跳，不能跳过尚未完成的减速。
            if (transition_end_mode_ != SetpointMode::HEARTBEAT) {
                startVelocityTransition(Eigen::Vector3d::Zero(), now);
                transition_end_mode_ = SetpointMode::HEARTBEAT;
            }
            return;
        }
        if (mode == SetpointMode::RAW_CTRL) {
            Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
            if (!(raw_target_.type_mask & mavros_msgs::PositionTarget::IGNORE_VX)) velocity.x() = raw_target_.velocity.x;
            if (!(raw_target_.type_mask & mavros_msgs::PositionTarget::IGNORE_VY)) velocity.y() = raw_target_.velocity.y;
            if (!(raw_target_.type_mask & mavros_msgs::PositionTarget::IGNORE_VZ)) velocity.z() = raw_target_.velocity.z;
            // 相同速度目标不重新计时，确保重复发布也能完成过渡。
            if (transition_end_mode_ == SetpointMode::RAW_CTRL &&
                (velocity - transition_velocity_).squaredNorm() < 1e-12) return;
            startVelocityTransition(velocity, now);
            transition_end_mode_ = SetpointMode::RAW_CTRL;
            return;
        }
        if (mode == SetpointMode::CONTROL) {
            setQuinticCurveTargetLocked(Eigen::Vector3d(target_pose_.pose.position.x,
                                                       target_pose_.pose.position.y,
                                                       target_pose_.pose.position.z), now);
            setpoint_mode_ = SetpointMode::CURVE_CTRL;
            return;
        }
    }
    if (mode != SetpointMode::CURVE_CTRL) velocity_transition_ = false;
    setpoint_mode_ = mode;
}

void MavKit::mavCalibrate(double mapx, double mapy, double factx, double facty) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    calib_.setMap(mapx, mapy);
    calib_.setFact(factx, facty);
    ROS_INFO("MavKit: Calibrated yaw_offset=%.4f rad (%.2f deg)",
             calib_.offset(), calib_.offset() * 180.0 / M_PI);
}

} // namespace maxt
