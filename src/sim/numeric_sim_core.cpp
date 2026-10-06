#include <maxt_pkg/numeric_sim.hpp>

#include <cmath>
#include <iomanip>

namespace maxt {

/**
 * @brief 建立模拟器的 ROS 接口，初始化默认状态，并按需创建 CSV 文件。
 */
NumericSim::NumericSim(ros::NodeHandle& nh, NumericSimConfig config)
    : nh_(nh), config_(std::move(config)) {
    // dt 决定数值积分的步长；speed_factor 只改变运行快慢。限制倍速
    // 可避免回调处理跟不上仿真时间，也避免非法浮点数污染整个状态。
    if (!std::isfinite(config_.dt) || config_.dt <= 0.0 ||
        !std::isfinite(config_.speed_factor) ||
        config_.speed_factor < 1.0 || config_.speed_factor > 20.0 ||
        !std::isfinite(config_.report_period) || config_.report_period < 0.0) {
        ROS_ERROR("NumericSim: invalid dt, speed_factor, or report_period.");
        return;
    }

    // 这些名称与真实 MAVROS 的标准话题保持一致，因此上层代码无需
    // 知道后端是真机还是 NumericSim。最后一个 true 表示 latched 发布。
    state_pub_ = nh_.advertise<mavros_msgs::State>("/mavros/state", 10, true);
    extended_state_pub_ =
        nh_.advertise<mavros_msgs::ExtendedState>("/mavros/extended_state", 10, true);
    pose_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, true);
    twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
        "/mavros/local_position/velocity_local", 10, true);
    clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 10, true);

    // 同时支持普通位姿目标和 RAW 目标。仿真器只使用“最后收到的
    // 那一类”命令，用 command_kind_ 记录这一选择。
    pose_setpoint_sub_ = nh_.subscribe("/mavros/setpoint_position/local", 10,
                                       &NumericSim::poseSetpointCb, this);
    raw_setpoint_sub_ = nh_.subscribe("/mavros/setpoint_raw/local", 10,
                                      &NumericSim::rawSetpointCb, this);
    arm_srv_ = nh_.advertiseService("/mavros/cmd/arming", &NumericSim::armCb, this);
    mode_srv_ = nh_.advertiseService("/mavros/set_mode", &NumericSim::modeCb, this);

    // 给出一个明确的初始飞控状态：已连接（可配置）、未解锁、
    // MANUAL、位于地面。单位四元数 (0,0,0,1) 表示无旋转。
    state_.connected = config_.connected;
    state_.armed = false;
    state_.mode = "MANUAL";
    extended_state_.landed_state = mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
    pose_.pose.orientation.w = 1.0;

    // CSV 是可选功能：路径为空时不打开文件，便于只做终端测试。
    // 列名中 sp_* 表示 setpoint，type_mask 仅对 RAW 命令有意义。
    if (!config_.csv_path.empty()) {
        csv_.open(config_.csv_path);
        if (!csv_.is_open()) {
            ROS_ERROR("NumericSim: cannot open CSV path: %s", config_.csv_path.c_str());
            return;
        }
        csv_ << "sim_time,x,y,z,vx,vy,vz,yaw,connected,armed,mode,landed,"
             << "command_kind,sp_x,sp_y,sp_z,sp_vx,sp_vy,sp_vz,"
             << "sp_ax,sp_ay,sp_az,sp_yaw,sp_yaw_rate,type_mask\n";
        csv_ << std::fixed << std::setprecision(6);
    }

    ready_ = true;
    ROS_INFO("NumericSim: dt=%.3f speed_factor=%.1f connected=%d csv=%s",
             config_.dt, config_.speed_factor, config_.connected,
             config_.csv_path.empty() ? "disabled" : config_.csv_path.c_str());
}

/**
 * @brief 在节点退出时报告终态，并确保缓冲的 CSV 数据被关闭和刷写。
 */
NumericSim::~NumericSim() {
    if (ready_) {
        ROS_INFO("NumericSim: stopped at t=%.2f pose=(%.2f, %.2f, %.2f) yaw=%.2f",
                 sim_time_, pose_.pose.position.x, pose_.pose.position.y,
                 pose_.pose.position.z, yaw_);
    }
    if (csv_.is_open()) {
        csv_.close();
    }
}

/**
 * @brief 返回模拟器是否通过参数和输出文件初始化。
 */
bool NumericSim::ready() const {
    return ready_;
}

/**
 * @brief 返回固定仿真步长按 speed_factor 换算后的墙上等待时间。
 */
double NumericSim::wallStepSeconds() const {
    return config_.dt / config_.speed_factor;
}

/**
 * @brief 保存普通位置 setpoint；下一仿真步会理想跟随其位置和姿态。
 */
void NumericSim::poseSetpointCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // ROS 消息使用 ConstPtr 避免回调入参拷贝；这里需要跨出回调保存，
    // 因此复制到成员变量。此时不立即更改状态，而是留到 step() 统一处理。
    pose_target_ = *msg;
    command_kind_ = CommandKind::POSE;
}

/**
 * @brief 保存 raw setpoint；下一仿真步按逐轴 type_mask 解释该命令。
 */
void NumericSim::rawSetpointCb(const mavros_msgs::PositionTarget::ConstPtr& msg) {
    raw_target_ = *msg;
    command_kind_ = CommandKind::RAW;
}

/**
 * @brief 模拟 MAVROS arming 服务，必要时由 accept_arm 静态故障开关拒绝。
 */
bool NumericSim::armCb(mavros_msgs::CommandBool::Request& req,
                       mavros_msgs::CommandBool::Response& res) {
    const bool accepted = config_.accept_arm;
    res.success = accepted;
    if (accepted) {
        state_.armed = req.value;
        ROS_INFO("NumericSim: arm request=%d accepted", req.value);
    } else {
        ROS_WARN("NumericSim: arm request=%d rejected by fault switch", req.value);
    }
    return true;
}

/**
 * @brief 模拟 MAVROS模式服务；仅 OFFBOARD 请求会受 accept_offboard 开关影响。
 */
bool NumericSim::modeCb(mavros_msgs::SetMode::Request& req,
                        mavros_msgs::SetMode::Response& res) {
    const bool accepted = req.custom_mode != "OFFBOARD" || config_.accept_offboard;
    res.mode_sent = accepted;
    if (accepted) {
        state_.mode = req.custom_mode;
        ROS_INFO("NumericSim: mode request=%s accepted", req.custom_mode.c_str());
    } else {
        ROS_WARN("NumericSim: OFFBOARD request rejected by fault switch");
    }
    return true;
}

/**
 * @brief 推进一个仿真步，并以固定顺序更新、发布、记录和报告状态。
 */
void NumericSim::step() {
    // 固定顺序很重要：先把时间推到 t+dt，再计算该时刻的状态，
    // 随后发布和记录同一份快照，从而保证 /clock、话题和 CSV 时间对齐。
    sim_time_ += config_.dt;
    updateState();
    publishState();
    writeCsv();
    reportIfDue();
}

}  // namespace maxt
