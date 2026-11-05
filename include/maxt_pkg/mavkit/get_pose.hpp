#ifndef MAVKIT_GET_POSE_HPP
#define MAVKIT_GET_POSE_HPP

#include <geometry_msgs/PoseStamped.h>

namespace maxt {
namespace pose_tools {

// 在同一原始坐标系中抵消 Home，保留当前消息的时间戳和姿态。
geometry_msgs::PoseStamped offsetHome(const geometry_msgs::PoseStamped& pose,
                                      const geometry_msgs::PoseStamped& home);

} // namespace pose_tools
} // namespace maxt

#endif
