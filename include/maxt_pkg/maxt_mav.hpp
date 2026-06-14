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
#include <mutex>
#include <atomic>

namespace maxt {

/**
 * @brief Setpoint 模式枚举
 */
enum class SetpointMode {
    HEARTBEAT,  // 心跳模式
    CONTROL,    // 位置控制模式
    RAW_CTRL,   // 原始设定值模式(setpoint_raw)
    STANDBY     // 待机模式
};

/**
 * @brief MavKit: 负责所有与 MAVROS 的底层交互
 * 特点：无阻塞、线程安全、不含 BT 逻辑
 */
class MavKit {
public:
    MavKit(ros::NodeHandle& nh);// 构造函数
    ~MavKit() = default;// 析构函数

    /**
     * @brief 初始化 MAVKit
     * @param nh ROS 节点句柄
     * @return true 初始化成功
     */
    bool mavInit(ros::NodeHandle& nh);

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
    void setRawTarget(const mavros_msgs::PositionTarget& target);

    /**
     * @brief 设置 setpoint 模式
     * @param mode Setpoint 模式
     */
    void setSetpointMode(SetpointMode mode);

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
    geometry_msgs::PoseStamped getCurrentPose() const;

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
    
    /**
     * @brief 检查是否到达（带参数，灵活度高）
     * @param x 目标 x 坐标
     * @param y 目标 y 坐标
     * @param z 目标 z 坐标
     * @param tolerance 容差值
     * @return true 到达
     */
    bool isReached(double x, double y, double z, double tolerance) const;

    // --- Home点管理 ---
    /**
     * @brief 获取无人机 Home点
     * @return geometry_msgs::PoseStamped Home点
     */ 
    geometry_msgs::PoseStamped getHomePose() const;

    /**
     * @brief 更新无人机 Home点
     */
    void updateHomePose();  

private:
    // 回调函数
    void stateCb(const mavros_msgs::State::ConstPtr& msg);
    void extendedStateCb(const mavros_msgs::ExtendedState::ConstPtr& msg);
    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void twistCb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void heartbeatTimerCallback(const ros::TimerEvent& event);

    ros::NodeHandle nh_;
    
    // 线程安全保障
    mutable std::mutex data_mtx_;
    
    // 底层数据
    mavros_msgs::State current_state_;
    mavros_msgs::ExtendedState current_extended_state_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::TwistStamped current_twist_;
    geometry_msgs::PoseStamped home_pose_;
    geometry_msgs::PoseStamped target_pose_;
    mavros_msgs::PositionTarget raw_target_;


    // ROS 句柄
    ros::Subscriber state_sub_, extended_state_sub_, pose_sub_, twist_sub_;
    ros::Publisher setpoint_pub_;
    ros::Publisher raw_setpoint_pub_;
    ros::ServiceClient arming_client_, mode_client_;
    ros::Timer heartbeat_timer_;

    // 状态标志位，用于判断是否已连接无人机
    std::atomic<bool> has_pose_;
    
    // Setpoint 模式
    std::atomic<SetpointMode> setpoint_mode_;
};

} // namespace maxt
#endif