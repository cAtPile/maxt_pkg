#include <maxt_pkg/numeric_sim.hpp>

#include <array>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace maxt {
namespace {

constexpr double kGroundZ = 0.0;
// 浮点计算很少精确等于 0，因此判断“已落地且静止”时使用容差。
constexpr double kGroundEpsilon = 1e-6;

/**
 * @brief 判断 PositionTarget 的给定字段是否被 type_mask 标记为忽略。
 */
bool ignored(uint16_t type_mask, uint16_t ignore_bit) {
    // type_mask 是位掩码：相应位为 1 表示“忽略该字段”，而不是启用。
    return (type_mask & ignore_bit) != 0;
}

/**
 * @brief 将角度限制到 (-pi, pi]，防止长时间 yaw_rate 积分失去数值可读性。
 */
double normalizeYaw(double yaw) {
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw <= -M_PI) yaw += 2.0 * M_PI;
    return yaw;
}

}  // namespace

/**
 * @brief 根据当前普通/RAW 命令更新飞行状态，并写回 yaw 四元数和接地状态。
 */
void NumericSim::updateState() {
    // 保存更新前的位置，POSE 命令直接跳到目标后，用有限差分
    // v = (p_new - p_old) / dt 反推这一步对应的速度。
    const std::array<double, 3> before{pose_.pose.position.x, pose_.pose.position.y,
                                       pose_.pose.position.z};

    if (command_kind_ == CommandKind::POSE) {
        // “理想跟随”：不设最大速度或加速度，在一个步长内到达目标。
        pose_.pose.position = pose_target_.pose.position;
        tf2::Quaternion q(pose_target_.pose.orientation.x, pose_target_.pose.orientation.y,
                          pose_target_.pose.orientation.z, pose_target_.pose.orientation.w);
        if (q.length2() > 1e-12) {
            // 上层发来的是四元数，本仿真只关心平面偏航 yaw，
            // 因此舍弃 roll/pitch。近似零四元数是非法姿态，遇到时保持旧 yaw。
            double roll, pitch;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_);
            yaw_ = normalizeYaw(yaw_);
        }
    } else if (command_kind_ == CommandKind::RAW) {
        applyRawAxis(0, mavros_msgs::PositionTarget::IGNORE_PX,
                     mavros_msgs::PositionTarget::IGNORE_VX, raw_target_.position.x,
                     raw_target_.velocity.x);
        applyRawAxis(1, mavros_msgs::PositionTarget::IGNORE_PY,
                     mavros_msgs::PositionTarget::IGNORE_VY, raw_target_.position.y,
                     raw_target_.velocity.y);
        applyRawAxis(2, mavros_msgs::PositionTarget::IGNORE_PZ,
                     mavros_msgs::PositionTarget::IGNORE_VZ, raw_target_.position.z,
                     raw_target_.velocity.z);
        updateYaw();
    }

    // 简单地面碰撞约束：z 不能低于 0，且接触地面后不允许继续向下。
    // 注意这不包含反弹、摩擦或起落架模型。
    if (pose_.pose.position.z <= kGroundZ) {
        pose_.pose.position.z = kGroundZ;
        if (twist_.twist.linear.z < 0.0) {
            twist_.twist.linear.z = 0.0;
        }
    }

    if (command_kind_ == CommandKind::POSE) {
        twist_.twist.linear.x = (pose_.pose.position.x - before[0]) / config_.dt;
        twist_.twist.linear.y = (pose_.pose.position.y - before[1]) / config_.dt;
        twist_.twist.linear.z = (pose_.pose.position.z - before[2]) / config_.dt;
    }

    // yaw_ 是姿态的单一真值来源。无论命令类型如何，最后都用它
    // 重建纯 yaw 四元数，保证发布的 pose 始终规范化。
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw_);
    pose_.pose.orientation.x = orientation.x();
    pose_.pose.orientation.y = orientation.y();
    pose_.pose.orientation.z = orientation.z();
    pose_.pose.orientation.w = orientation.w();
    updateLandedState();
}

/**
 * @brief 对一个位置轴执行最小 setpoint 规则：位置直接跟随优先于速度积分。
 */
void NumericSim::applyRawAxis(std::size_t axis, uint16_t position_ignore,
                              uint16_t velocity_ignore, double position_target,
                              double velocity_target) {
    // 通过指针把 x/y/z 三轴共用同一套规则，避免三份重复逻辑。
    // 该函数只从 updateState() 传入 0、1、2，因此 else 分支代表 z 轴。
    double* position = nullptr;
    double* velocity = nullptr;
    if (axis == 0) {
        position = &pose_.pose.position.x;
        velocity = &twist_.twist.linear.x;
    } else if (axis == 1) {
        position = &pose_.pose.position.y;
        velocity = &twist_.twist.linear.y;
    } else {
        position = &pose_.pose.position.z;
        velocity = &twist_.twist.linear.z;
    }

    const bool has_position = !ignored(raw_target_.type_mask, position_ignore);
    const bool has_velocity = !ignored(raw_target_.type_mask, velocity_ignore);
    const double before = *position;

    // 优先级和数值规则：
    // 1. 位置有效：直接到目标；速度也有效时相信命令值，否则用位置差反推。
    // 2. 只有速度有效：用显式欧拉法 p <- p + v*dt 积分。
    // 3. 两者都忽略：位置保持，将报告速度清零。
    // acceleration_or_force 在本最小模型中只记录到 CSV，不参与积分。
    if (has_position) {
        *position = position_target;
        *velocity = has_velocity ? velocity_target : (*position - before) / config_.dt;
    } else if (has_velocity) {
        *velocity = velocity_target;
        *position += *velocity * config_.dt;
    } else {
        *velocity = 0.0;
    }
}

/**
 * @brief 按 type_mask 选择直接设置 yaw 或积分 yaw_rate；都被忽略时保持当前值。
 */
void NumericSim::updateYaw() {
    // yaw 和 yaw_rate 同时有效时，绝对 yaw 优先；这与位置优先于
    // 速度的简化策略一致。yaw_rate 的单位是 rad/s。
    if (!ignored(raw_target_.type_mask, mavros_msgs::PositionTarget::IGNORE_YAW)) {
        yaw_ = normalizeYaw(raw_target_.yaw);
    } else if (!ignored(raw_target_.type_mask,
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE)) {
        yaw_ = normalizeYaw(yaw_ + raw_target_.yaw_rate * config_.dt);
    }
}

/**
 * @brief 从高度和垂速推导接地状态，并允许 report_landed 故障开关强制不接地。
 */
void NumericSim::updateLandedState() {
    // 同时检查高度和竖直速度，避免“刚接触地面但仍在运动”被过早
    // 判定为完成降落。report_landed=false 可用于测试上层的降落超时逻辑。
    const bool on_ground = pose_.pose.position.z <= kGroundEpsilon &&
                           std::fabs(twist_.twist.linear.z) <= kGroundEpsilon;
    extended_state_.landed_state =
        (config_.report_landed && on_ground)
            ? mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND
            : mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
}

}  // namespace maxt
