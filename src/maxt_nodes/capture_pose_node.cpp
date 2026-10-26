#include <maxt_pkg/maxt_nodes/capture_pose_node.hpp>

namespace maxt {

BT::PortsList CapturePoseNode::providedPorts() {
    return {
        BT::InputPort<double>("height_offset", 0.0, "height added to captured Z (m)"),
        BT::OutputPort<double>("x"),
        BT::OutputPort<double>("y"),
        BT::OutputPort<double>("z")
    };
}

BT::NodeStatus CapturePoseNode::onStart() {
    // 每次启动从节点私有命名空间读取本模块的 ROS 参数。
    ros::NodeHandle params("~capture_pose");
    params.param("timeout", timeout_, 10.0);

    getInput("height_offset", height_offset_);
    started_ = ros::WallTime::now();
    return onRunning();
}

BT::NodeStatus CapturePoseNode::onRunning() {
    const auto pose = mav_.getCurrentPose();
    // MavKit 初始化时尚未收到定位，不能把默认零坐标当成起飞点。
    if (!pose.header.stamp.isZero()) {
        setOutput("x", pose.pose.position.x);
        setOutput("y", pose.pose.position.y);
        setOutput("z", pose.pose.position.z + height_offset_);
        return BT::NodeStatus::SUCCESS;
    }
    if ((ros::WallTime::now() - started_).toSec() >= timeout_) {
        ROS_ERROR("CapturePose: timeout waiting for drone pose");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

} // namespace maxt
