#include <maxt_pkg/maxt_nodes/base_point_azimuth_fix_node.hpp>

namespace maxt {

BasePointAzimuthFixNode::BasePointAzimuthFixNode(
    const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
    : BT::SyncActionNode(name, config), mav_(mav) {}

BT::PortsList BasePointAzimuthFixNode::providedPorts() {
    return {BT::OutputPort<double>("bp_x_raw"),
            BT::OutputPort<double>("bp_y_raw"),
            BT::OutputPort<double>("bp_z_raw")};
}

BT::NodeStatus BasePointAzimuthFixNode::tick() {
    const auto pose = mav_.GetRawPose();
    setOutput("bp_x_raw", pose.pose.position.x);
    setOutput("bp_y_raw", pose.pose.position.y);
    setOutput("bp_z_raw", pose.pose.position.z);
    return BT::NodeStatus::SUCCESS;
}

} // namespace maxt
