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
    setpoint_mode_ = mode;
    //ROS_INFO("Setpoint mode set to: %d", static_cast<int>(mode));
}

void MavKit::mavCalibrate(double mapx, double mapy, double factx, double facty) {
    calib_.setMap(mapx, mapy);
    calib_.setFact(factx, facty);
    ROS_INFO("MavKit: Calibrated yaw_offset=%.4f rad (%.2f deg)",
             calib_.offset(), calib_.offset() * 180.0 / M_PI);
}

} // namespace maxt