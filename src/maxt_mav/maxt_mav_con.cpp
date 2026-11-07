/**
 * @file maxt_mav.cpp
 * @brief maxt的mav组件实现
 * @date 4-14
 */
#include <maxt_pkg/mavkit/maxt_mav.hpp>
#include <cmath>

namespace maxt {

MavKit::MavKit(ros::NodeHandle& nh) : nh_(nh), audio_(nh) {
    playAudio("cn-welcome.wav");
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
    nh_.param<double>("nav/stop_velocity_tolerance", stop_velocity_tolerance_, 0.1);
    nh_.param<double>("nav/stop_acceleration", stop_acceleration_, 1.0);
    nh_.param<double>("nav/stop_settle_time", stop_settle_time_, 0.4);
    if (!std::isfinite(slow_down_time_) || slow_down_time_ <= 0.0 ||
        !std::isfinite(stop_acceleration_) || stop_acceleration_ <= 0.0 ||
        !std::isfinite(stop_velocity_tolerance_) || stop_velocity_tolerance_ <= 0.0 ||
        !std::isfinite(stop_settle_time_) || stop_settle_time_ <= 0.0) {
        ROS_ERROR("Invalid nav stop limits: all values must be finite and positive.");
        return false;
    }

    nh_.param<double>("reach_check/vertical_alt_tolerance", vertical_alt_tolerance_, 0.1);
    nh_.param<double>("reach_check/vertical_vel_tolerance", vertical_vel_tolerance_, 0.2);
    nh_.param<double>("reach_check/position_pose_tolerance", position_pose_tolerance_, 0.1);
    nh_.param<double>("reach_check/position_vel_tolerance", position_vel_tolerance_, 0.2);

    nh_.param<double>("reach_check/global_position_tolerance", global_position_tolerance_, 1.0);
    nh_.param<double>("reach_check/global_altitude_tolerance", global_altitude_tolerance_, 0.5);

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

    global_pose_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>(
        "/mavros/global_position/global", 1, &MavKit::globalPoseCb, this);
    global_altitude_sub_ = nh_.subscribe<std_msgs::Float64>(
        "/mavros/global_position/rel_alt", 1, &MavKit::globalAltitudeCb, this);
    global_raw_setpoint_pub_ = nh_.advertise<mavros_msgs::GlobalPositionTarget>(
        "/mavros/setpoint_raw/global", 10);

    // 4. 初始化 Service 客户端
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 5. 启动心跳定时器
    heartbeat_timer_ = nh_.createTimer(
        ros::Duration(1.0 / heartbeat_rate), &MavKit::heartbeatTimerCallback, this);

    ROS_INFO("MavKit initialized. Waiting for MAVROS connection...");
    return true;
}

void MavKit::mavCalibrate(double mapx, double mapy, double factx, double facty) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    calib_.setMap(mapx, mapy);
    calib_.setFact(factx, facty);
    ROS_INFO("MavKit: Calibrated yaw_offset=%.4f rad (%.2f deg)",
             calib_.offset(), calib_.offset() * 180.0 / M_PI);
}

} // namespace maxt
