#include <maxt_pkg/maxt_nodes/connect_check_node.hpp>
#include <Eigen/Geometry>
#include <cmath>

namespace maxt {

BT::PortsList ConnectCheckNode::providedPorts() {
    return {
        BT::InputPort<double>("map_x",  0.0,  "reference map x (world)"),
        BT::InputPort<double>("map_y",  0.0,  "reference map y (world)"),
        BT::InputPort<double>("fact_x", 0.0,  "reference SLAM x"),
        BT::InputPort<double>("fact_y", 0.0,  "reference SLAM y")
    };
}

ConnectCheckNode::ConnectCheckNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav) {
}

BT::NodeStatus ConnectCheckNode::onStart() {
    // 每次启动从节点私有命名空间读取本模块的 ROS 参数。
    ros::NodeHandle params("~connect_check");
    params.param("timeout", timeout_, 60.0);

    getInput<double>("map_x", map_x_);
    getInput<double>("map_y", map_y_);
    getInput<double>("fact_x", fact_x_);
    getInput<double>("fact_y", fact_y_);

    start_time_ = ros::WallTime::now();
    ROS_INFO("ConnectCheckNode: Waiting for drone connection...");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ConnectCheckNode::onRunning() {
    if ((ros::WallTime::now() - start_time_).toSec() > timeout_) {
        ROS_ERROR("ConnectCheckNode: Connection timeout!");
        return BT::NodeStatus::FAILURE;
    }

    if (mav_.isConnected()) {
        if (map_x_ != 0.0 || map_y_ != 0.0) {
            mav_.mavCalibrate(map_x_, map_y_, fact_x_, fact_y_);
        } else {
            const auto pose = mav_.getCurrentPose();
            const auto& orientation = pose.pose.orientation;
            Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
            const double norm = q.norm();
            if (!std::isfinite(norm) || norm < 1e-6) {
                ROS_INFO_THROTTLE(1.0, "ConnectCheckNode: Waiting for valid local pose...");
                return BT::NodeStatus::RUNNING;
            }
            q.normalize();
            const double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
            // 将任务 +X 固定到连接时的机头水平朝向，只标定一次。
            mav_.mavCalibrate(1.0, 0.0, std::cos(yaw), std::sin(yaw));
        }
        ROS_INFO("ConnectCheckNode: Drone connected.");
        return BT::NodeStatus::SUCCESS;
    }

    ROS_INFO_THROTTLE(1.0, "ConnectCheckNode: Waiting for drone connection...");
    return BT::NodeStatus::RUNNING;
}

void ConnectCheckNode::onHalted() {
    ROS_WARN("ConnectCheckNode: Halted during connection check!");
}

} // namespace maxt
