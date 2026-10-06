#ifndef MAXT_NUMERIC_SIM_HPP
#define MAXT_NUMERIC_SIM_HPP

#include <fstream>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

namespace maxt {

/**
 * @brief 纯终端数值仿真的固定配置与静态故障开关。
 *
 * 这些参数在节点启动时由 ROS 私有参数读入，运行期间不会动态更改。
 * 其中 accept_* 和 report_landed 不是真实飞行器参数，而是用来复现失败
 * 分支的“故障注入开关”。
 */
struct NumericSimConfig {
    /// 仿真步长（秒）；每次 step() 使仿真时间前进该值。
    double dt{0.02};
    /// 仿真倍速；墙上等待时间为 dt / speed_factor。
    double speed_factor{1.0};
    /// 终端状态摘要的仿真时间周期（秒）；0 表示禁用。
    double report_period{1.0};
    /// CSV 记录路径；空字符串表示不记录。
    std::string csv_path{"/tmp/maxt_numeric_sim.csv"};
    /// 发布的 mavros_msgs/State.connected 值。
    bool connected{true};
    /// 是否接受解锁/上锁服务请求。
    bool accept_arm{true};
    /// 是否接受 OFFBOARD 模式请求。
    bool accept_offboard{true};
    /// false 时即使高度为 0，也始终报告“空中”。
    bool report_landed{true};
};

/**
 * @brief 提供最小 MAVROS 接口、仿真时间和理想 setpoint 跟随的仿真器。
 *
 * 数据流可以理解为：
 *
 *   行为树/MavKit --setpoint--> NumericSim --pose/state/clock--> 行为树/MavKit
 *
 * 该类刻意不模拟电机、惯性、阻力或飞控内环。它将 position/raw setpoint
 * 直接转换为可供 MavKit 读取的理想状态，目的是在无 PX4/Gazebo 时验证
 * 行为树流程和 MAVROS 接口交互，不用于评估真实飞行性能。
 */
class NumericSim {
public:
    /** @brief 初始化 ROS 通信、默认飞行状态与可选 CSV 输出。 */
    NumericSim(ros::NodeHandle& nh, NumericSimConfig config);
    /** @brief 输出终态摘要并关闭 CSV 文件。 */
    ~NumericSim();

    /** @brief 返回构造和输出文件初始化是否成功。 */
    bool ready() const;
    /** @brief 推进一个固定仿真步，依次更新状态、发布数据和记录输出。 */
    void step();
    /** @brief 返回当前倍速下单个仿真步对应的墙上等待时间。 */
    double wallStepSeconds() const;

private:
    /**
     * @brief 最近一次收到的控制命令类型。
     *
     * POSE 对应 /mavros/setpoint_position/local；RAW 对应
     * /mavros/setpoint_raw/local。后到的命令会覆盖先前命令的主导权。
     */
    enum class CommandKind { NONE, POSE, RAW };

    /** @brief 缓存普通位置 setpoint，并将其设为当前有效命令。 */
    void poseSetpointCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    /** @brief 缓存 raw PVA setpoint，并将其设为当前有效命令。 */
    void rawSetpointCb(const mavros_msgs::PositionTarget::ConstPtr& msg);
    /** @brief 响应 arm/disarm 服务，并应用 accept_arm 故障开关。 */
    bool armCb(mavros_msgs::CommandBool::Request& req,
               mavros_msgs::CommandBool::Response& res);
    /** @brief 响应模式切换服务，并应用 accept_offboard 故障开关。 */
    bool modeCb(mavros_msgs::SetMode::Request& req,
                mavros_msgs::SetMode::Response& res);

    /** @brief 根据当前命令更新位置、速度、yaw 和接地状态。 */
    void updateState();
    /** @brief 发布 /clock 和四类供 MavKit 消费的 MAVROS 状态。 */
    void publishState();
    /** @brief 将当前状态、最新命令和 setpoint 写入一行 CSV。 */
    void writeCsv();
    /** @brief 到达报告周期时打印一条低频终端摘要。 */
    void reportIfDue();
    /** @brief 按单轴 type_mask 规则执行位置直接跟随或速度积分。 */
    void applyRawAxis(std::size_t axis, uint16_t position_ignore,
                      uint16_t velocity_ignore, double position_target,
                      double velocity_target);
    /** @brief 解释 raw setpoint 中的 yaw 或 yaw_rate 字段。 */
    void updateYaw();
    /** @brief 根据地面约束和 report_landed 开关更新 ExtendedState。 */
    void updateLandedState();
    /** @brief 返回当前命令对应轴的位置目标，供 CSV 输出使用。 */
    double targetValue(std::size_t axis) const;
    /** @brief 返回当前命令对应轴的速度目标，供 CSV 输出使用。 */
    double targetVelocity(std::size_t axis) const;
    /** @brief 返回当前命令对应轴的加速度目标，供 CSV 输出使用。 */
    double targetAcceleration(std::size_t axis) const;
    /** @brief 返回当前命令类别的稳定文本名称。 */
    std::string commandKindName() const;

    ros::NodeHandle nh_;
    NumericSimConfig config_;
    /// false 表示参数校验或 CSV 初始化失败，节点应立即退出。
    bool ready_{false};

    // 向被测系统伪装成 MAVROS 的发布器。latched=true 使新订阅者立即
    // 拿到最近状态，不必等到下一个仿真步。
    ros::Publisher state_pub_;
    ros::Publisher extended_state_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher twist_pub_;
    ros::Publisher clock_pub_;
    ros::Subscriber pose_setpoint_sub_;
    ros::Subscriber raw_setpoint_sub_;
    ros::ServiceServer arm_srv_;
    ros::ServiceServer mode_srv_;

    // state_/extended_state_ 表示飞控状态；pose_/twist_ 表示仿真器的当前运动状态。
    mavros_msgs::State state_;
    mavros_msgs::ExtendedState extended_state_;
    geometry_msgs::PoseStamped pose_;
    geometry_msgs::TwistStamped twist_;
    geometry_msgs::PoseStamped pose_target_;
    mavros_msgs::PositionTarget raw_target_;
    CommandKind command_kind_{CommandKind::NONE};

    /// 由 step() 离散推进的 ROS 仿真时间，与机器的真实墙上时间分离。
    double sim_time_{0.0};
    /// 内部用弧度保存的偏航角，发布 pose 时再转成四元数。
    double yaw_{0.0};
    /// 上次打印状态摘要时的仿真时间。
    double last_report_time_{-1.0};
    std::ofstream csv_;
};

}  // namespace maxt

#endif  // MAXT_NUMERIC_SIM_HPP
