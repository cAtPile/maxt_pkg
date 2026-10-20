#ifndef MAXT_CAPTURE_POSE_NODE_HPP
#define MAXT_CAPTURE_POSE_NODE_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

// 等待定位，并将一次位置快照写到黑板，用于返回实际起飞点。
class CapturePoseNode : public MavActionNode {
public:
    using MavActionNode::MavActionNode;
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}

private:
    ros::WallTime started_;
    double timeout_{10.0};
    double height_offset_{0.0};
};

} // namespace maxt
#endif
