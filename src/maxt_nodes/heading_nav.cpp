#include <maxt_pkg/maxt_nodes/heading_nav_node.hpp>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace maxt {

HeadingNavNode::HeadingNavNode(const std::string& name,
                             const BT::NodeConfiguration& config,
                             MavKit& mav, const ros::NodeHandle& nh)
    : MavActionNode(name, config, mav), nh_(nh) {}

BT::PortsList HeadingNavNode::providedPorts() {
    return {
        BT::InputPort<double>("heading_angle", "travel heading in control frame (rad)"),
        BT::InputPort<double>("nav_dis", "horizontal travel distance (m)"),
        BT::InputPort<double>("tolerance", 0.1, "arrival tolerance (m)"),
        BT::InputPort<double>("timeout", 60.0, "timeout (s)")
    };
}

BT::NodeStatus HeadingNavNode::onStart() {
    double heading_angle, nav_dis;
    if (!getInput("heading_angle", heading_angle) || !getInput("nav_dis", nav_dis)) {
        ROS_ERROR("[HeadingNav] Missing heading_angle or nav_dis!");
        return BT::NodeStatus::FAILURE;
    }
    double tolerance = 0.1;
    double timeout = 60.0;
    getInput("tolerance", tolerance);
    getInput("timeout", timeout);

    const auto pose = mav_.getCurrentPose();
    const auto home = mav_.getHomePose();
    // Nav 接收相对 home 的坐标；航向相对控制坐标系 +X，逆时针为正。
    const double x = pose.pose.position.x + nav_dis * std::cos(heading_angle);
    const double y = pose.pose.position.y + nav_dis * std::sin(heading_angle);
    const double z = pose.pose.position.z;
    const auto to_string = [](double value) {
        std::ostringstream stream;
        stream << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
        return stream.str();
    };

    BT::NodeConfiguration nav_config;
    nav_config.blackboard = config().blackboard;
    nav_config.input_ports = {
        {"x", to_string(x - home.pose.position.x)},
        {"y", to_string(y - home.pose.position.y)},
        {"z", to_string(z - home.pose.position.z)},
        {"tolerance", to_string(tolerance)},
        {"timeout", to_string(timeout)}
    };
    // 每次启动重新捕获起点；复用 Nav 的曲线、PID、超时及中断处理。
    nav_ = std::make_unique<NavNode>(name() + "_nav", nav_config, mav_, nh_);
    ROS_INFO("[HeadingNav] heading=%.3f rad distance=%.2f target=(%.2f, %.2f, %.2f)",
             heading_angle, nav_dis, x, y, z);
    return nav_->executeTick();
}

BT::NodeStatus HeadingNavNode::onRunning() {
    return nav_->executeTick();
}

void HeadingNavNode::onHalted() {
    nav_->halt();
}

} // namespace maxt
