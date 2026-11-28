#include <maxt_pkg/maxt_nodes/nav/hight_ref_to_local.hpp>

namespace maxt {

BT::PortsList HightRef2Local::providedPorts() {
    return {BT::InputPort<double>("home_alt"), BT::InputPort<double>("target_hight"),
            BT::OutputPort<double>("z")};
}

BT::NodeStatus HightRef2Local::tick() {
    double reference, offset;
    // home_alt 为起飞点的 local Z，offset 为相对起飞点的目标高度。
    if (!getInput("home_alt", reference) || !getInput("target_hight", offset) ||
        !setOutput("z", reference + offset)) {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

} // namespace maxt
