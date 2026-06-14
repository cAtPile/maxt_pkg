# MavKit API 参考

## 头文件

```cpp
#include <maxt_pkg/maxt_mav.hpp>
```

## 指令方法 (Commands)

| 方法 | 说明 |
|------|------|
| `void requestArm(bool arm)` | 请求解锁/加锁 |
| `void requestMode(const std::string& mode)` | 请求飞行模式，通常 `"OFFBOARD"` |
| `void setTargetPose(double x, double y, double z)` | 设置位置目标 (setpoint_position) |
| `void setRawTarget(const mavros_msgs::PositionTarget& target)` | 设置原始目标 (setpoint_raw)，支持速度+位置混合控制 |
| `void setSetpointMode(SetpointMode mode)` | 切换 setpoint 模式 |

## 状态查询 (Queries)

| 方法 | 返回类型 | 说明 |
|------|----------|------|
| `bool isConnected()` | bool | FCU 连接状态 |
| `bool isArmed()` | bool | 解锁状态 |
| `bool isLand()` | bool | 接地状态 (landed_state == ON_GROUND) |
| `std::string getMode()` | string | 当前飞行模式 |
| `PoseStamped getCurrentPose()` | PoseStamped | 当前位置 (map 坐标系) |
| `TwistStamped getCurrentTwist()` | TwistStamped | 当前速度 (NED) |
| `double get_current_yaw()` | double | 当前偏航角 (rad) |
| `bool isReached(x, y, z, tol)` | bool | 到达目标位置判断 |
| `PoseStamped getHomePose()` | PoseStamped | Home 点 |
| `void updateHomePose()` | void | 刷新 Home 点 |

## SetpointMode 枚举

```cpp
enum class SetpointMode {
    HEARTBEAT,  // 心跳模式：发布当前位置，维持 OFFBOARD
    CONTROL,    // 位置控制模式：setpoint_position
    RAW_CTRL,   // 原始控制模式：setpoint_raw (position + velocity + acceleration)
    STANDBY     // 待机模式：不发布
};
```

## 典型用法

### 解锁和切 OFFBOARD (Takeoff 模式)

```cpp
// 解锁
if (!mav_.isArmed()) {
    mav_.requestArm(true);
    return RUNNING;
}
// 切换 OFFBOARD
if (mav_.getMode() != "OFFBOARD") {
    mav_.setSetpointMode(SetpointMode::HEARTBEAT);
    mav_.requestMode("OFFBOARD");
    return RUNNING;
}
```

### 速度控制上升 (RAW_CTRL 模式)

```cpp
mavros_msgs::PositionTarget target;
target.header.frame_id = "map";
target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
target.type_mask =
    IGNORE_PZ | IGNORE_VX | IGNORE_VY |
    IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ |
    IGNORE_YAW_RATE;
target.position.x = hold_x;
target.position.y = hold_y;
target.velocity.z = ascent_speed;
target.yaw = hold_yaw;
mav_.setRawTarget(target);
mav_.setSetpointMode(SetpointMode::RAW_CTRL);
```

### PositionTarget type_mask 说明

使用 `IGNORE_*` 标志选择要控制的通道：

| 标志 | 含义 |
|------|------|
| `IGNORE_PX/PY/PZ` | 忽略位置 |
| `IGNORE_VX/VY/VZ` | 忽略速度 |
| `IGNORE_AFX/AFY/AFZ` | 忽略加速度/力 |
| `IGNORE_YAW` | 忽略偏航角 |
| `IGNORE_YAW_RATE` | 忽略偏航角速率 |

常见组合: 忽略 PZ + 所有速度 + 所有加速度 + yaw_rate = 位置XY锁定 + 速度Z控制
