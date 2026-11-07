/**
 * @file maxt_mav.hpp
 * @brief maxt的mav组件
 * @date 4-14
 */
#ifndef MAXT_MAV_HPP
#define MAXT_MAV_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <maxt_pkg/calibrate_tools.hpp>
#include <maxt_pkg/quintic_curve.hpp>
#include <maxt_pkg/audio_caller.hpp>
#include <Eigen/Core>
#include <mutex>
#include <atomic>

namespace maxt {
namespace pose_tools {

// 在同一原始坐标系中抵消 Home，保留当前消息的时间戳和姿态。
geometry_msgs::PoseStamped offsetHome(const geometry_msgs::PoseStamped& pose,
                                      const geometry_msgs::PoseStamped& home);

} // namespace pose_tools

/**
 * @brief Setpoint 模式枚举
 */
enum class SetpointMode {
    HEARTBEAT,  // 心跳模式
    CONTROL,    // 位置控制模式
    RAW_CTRL,   // 原始设定值模式(setpoint_raw)
    STANDBY,    // 待机模式
    CURVE_CTRL, // 定时器采样五次曲线并发布 p/v/a
    GLOBAL_RAW_CTRL, // 全球经纬度目标
    SLOW_DOWN   // 一次性停止曲线，实测低速稳定后进入心跳
};

// 世界坐标系下的曲线参考状态，不由传感器逐帧覆盖。
struct CurveState {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
};

/**
 * @brief MavKit: 负责所有与 MAVROS 的底层交互
 * 特点：无阻塞、线程安全、不含 BT 逻辑
 */
class MavKit {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MavKit(ros::NodeHandle& nh);// 构造函数
    ~MavKit() = default;// 析构函数

    /**
     * @brief 初始化 MAVKit
     * @param nh ROS 节点句柄
     * @return true 初始化成功
     */
    bool mavInit(ros::NodeHandle& nh);

    // 尽力提交播报，不等待音频服务或播放结果。
    void playAudio(const std::string& filename) { audio_.play(filename); }

    /**
     * @brief 标定：根据参照点计算 SLAM 坐标系偏航偏移
     * @param mapx 地图(测绘)坐标 x
     * @param mapy 地图(测绘)坐标 y
     * @param factx SLAM 实际读取坐标 x
     * @param facty SLAM 实际读取坐标 y
     */
    void mavCalibrate(double mapx, double mapy, double factx, double facty);

    // --- 指令触发 (Trigger) ---
    /**
     * @brief 请求无人机 arming 状态
     * @param arm 是否 arming
     */
    void requestArm(bool arm);

    /**
     * @brief 请求无人机飞行模式
     * @param mode 飞行模式字符串
     */
    void requestMode(const std::string& mode);

    /**
     * @brief 设置无人机目标位姿
     * @param x 目标 x 坐标
     * @param y 目标 y 坐标
     * @param z 目标 z 坐标
     */
    void setTargetPose(double x, double y, double z);

    /**
     * @brief 设置原始目标值，通过 /mavros/setpoint_raw/local 发布
     * @param target 原始目标 (PositionTarget message)
     */
    // apply_default_yaw=false permits explicit yaw/yaw-rate control.
    void setRawTarget(const mavros_msgs::PositionTarget& target, bool apply_default_yaw = true);

    // 全球目标直接发布；FRAME_GLOBAL_INT 的高度为海拔米。
    void SetGlobalRawTarget(const mavros_msgs::GlobalPositionTarget& target);
    // 经纬度及飞控相对 Home 高度，FRAME_GLOBAL_REL_ALT。
    mavros_msgs::GlobalPositionTarget GetGlobalPose() const;
    bool hasGlobalPose() const;
    // 水平按经纬度测地距离检查，高度按原始 local Z 检查。
    bool ReachGlobalCheck(const mavros_msgs::GlobalPositionTarget& target,
                          double altitude_raw) const;

    // 设置世界坐标系目标，终点速度、加速度为零；随后切换到 CURVE_CTRL。
    // raw_local=true：目标、曲线状态和发布均使用原始 local 坐标。
    void setQuinticCurveTarget(const Eigen::Vector3d& position, bool raw_local = false);
    double getCurveTime() const;

    /**
     * @brief 设置 setpoint 模式
     * @param mode Setpoint 模式
     */
    void setSetpointMode(SetpointMode mode);
    bool isSlowingDown() const { return setpoint_mode_ == SetpointMode::SLOW_DOWN; }

    // --- 状态查询 (Queries) ---
    /**
     * @brief 检查无人机是否已连接
     * @return true 已连接
     */
    bool isConnected() const;

    /**
     * @brief 检查无人机是否已 arming
     * @return true 已 arming
     */
    bool isArmed() const;

    /**
     * @brief 检查无人机是否已降落（接地）
     * @return true 已接地 (landed_state == LANDED_STATE_ON_GROUND)
     */
    bool isLand() const;

    /**
     * @brief 获取无人机当前飞行模式
     * @return 飞行模式字符串
     */
    std::string getMode() const;
    
    /**
     * @brief 获取无人机当前位姿
     * @return geometry_msgs::PoseStamped 当前位姿
     */
    // 原始 MAVROS 坐标；以下查询均返回持锁复制的快照。
    geometry_msgs::PoseStamped GetRawPose() const;
    // 起飞前保存原始位姿，独立于用于坐标偏移的 Home。
    void RecordRawHomePose();
    // 返回起飞前记录的原始位姿快照，不做偏移或标定。
    geometry_msgs::PoseStamped GetRawHomePose() const;
    // 原始坐标减去 Home 的 XYZ，不做角度标定。
    geometry_msgs::PoseStamped GetHomeOffsetPose() const;
    // 先抵消 Home，再将 XY 逆向旋转到工程世界坐标。
    geometry_msgs::PoseStamped GetHomeCalibratePose() const;
    // 仅标定 XY，不抵消 Home；保持既有导航坐标语义。
    geometry_msgs::PoseStamped GetCalibratePose() const;

    /**
     * @brief 获取无人机当前速度
     * @return geometry_msgs::TwistStamped 当前速度 (NED)
     */
    geometry_msgs::TwistStamped getCurrentTwist() const;

    /**
     * @brief 获取无人机当前偏航角
     * @return double 偏航角 (rad)
     */
    double get_current_yaw() const;

    // 起飞前锁定 MAVROS 本地 ENU 偏航，供后续设点统一保持。
    void setDefaultYaw(double yaw);
    
    /**
     * @brief 检查是否到达（带参数，灵活度高）
     * @param x 目标 x 坐标
     * @param y 目标 y 坐标
     * @param z 目标 z 坐标
     * @param tolerance 容差值
     * @return true 到达
     */
    bool isReached(double x, double y, double z, double tolerance) const;

    // 原始局部坐标下的到达检查，要求位置及零速度误差同时满足配置容差。
    bool ReachAltitudeCheck(double altitude_raw) const;
    bool ReachPoseCheck(geometry_msgs::PoseStamped pose_raw) const;

    // --- Home点管理 ---
    /**
     * @brief 获取经过标定矫正的无人机 Home点
     * @return geometry_msgs::PoseStamped 世界坐标系下的 Home点
     */ 
    geometry_msgs::PoseStamped getHomePose() const;

    /**
     * @brief 更新无人机 Home点
     */
    void updateHomePose();  

private:
    // ROS 订阅回调，统一实现在 src/mavkit/callback.cpp。
    void stateCb(const mavros_msgs::State::ConstPtr& msg);
    void extendedStateCb(const mavros_msgs::ExtendedState::ConstPtr& msg);
    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void twistCb(const geometry_msgs::TwistStamped::ConstPtr& msg);

    void globalPoseCb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void globalAltitudeCb(const std_msgs::Float64::ConstPtr& msg);

    // 定时器回调，实现在 src/mavkit/timer.cpp。
    void heartbeatTimerCallback(const ros::TimerEvent& event);

    // 调用方持有 data_mtx_，采样当前曲线并填充原始参考设点。
    void loadCurveRaw(const ros::Time& now);
    void setQuinticCurveTargetLocked(const Eigen::Vector3d& position, const ros::Time& now);
    void startStopCurve(const ros::Time& now);
    void captureCurveState(const ros::Time& now);

    ros::NodeHandle nh_;
    AudioCaller audio_;
    
    // 线程安全保障
    mutable std::mutex data_mtx_;
    
    // 底层数据
    mavros_msgs::State current_state_;
    mavros_msgs::ExtendedState current_extended_state_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::TwistStamped current_twist_;
    geometry_msgs::PoseStamped home_pose_;
    geometry_msgs::PoseStamped raw_home_pose_;
    geometry_msgs::PoseStamped target_pose_;
    double default_yaw_{0.0};
    bool has_default_yaw_{false};
    mavros_msgs::PositionTarget raw_target_;
    mavros_msgs::GlobalPositionTarget global_target_;
    mavros_msgs::GlobalPositionTarget global_pose_;
    bool has_global_fix_{false};
    bool has_global_altitude_{false};
    mavros_msgs::PositionTarget curve_raw_target_;
    CurveState curve_state_;
    bool curve_raw_local_{false};
    QuinticCurve curve_;
    ros::Time curve_start_;
    double curve_time_{1.0};
    bool velocity_transition_{false};
    double slow_down_time_{1.0};
    double stop_acceleration_{1.0};
    double stop_settle_time_{0.4};
    ros::Time stop_stable_since_;
    double stop_velocity_tolerance_{0.1};


    // ROS 句柄
    ros::Subscriber state_sub_, extended_state_sub_, pose_sub_, twist_sub_;
    ros::Subscriber global_pose_sub_, global_altitude_sub_;
    ros::Publisher global_raw_setpoint_pub_;
    ros::Publisher setpoint_pub_;
    ros::Publisher raw_setpoint_pub_;
    ros::ServiceClient arming_client_, mode_client_;
    ros::Timer heartbeat_timer_;

    // 状态标志位，用于判断是否已连接无人机
    std::atomic<bool> has_pose_;
    
    // Setpoint 模式
    std::atomic<SetpointMode> setpoint_mode_;

    // 到达检查容差：位置 m，速度 m/s。
    double vertical_alt_tolerance_{0.1};
    double vertical_vel_tolerance_{0.2};
    double position_pose_tolerance_{0.1};
    double position_vel_tolerance_{0.2};

    double global_position_tolerance_{1.0};
    double global_altitude_tolerance_{0.5};

    // 标定工具
    CalibrateTools calib_;
};

} // namespace maxt
#endif
