#pragma once

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class Heading2Local : public BT::SyncActionNode {
public:
    Heading2Local(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
        : BT::SyncActionNode(name, config), mav_(mav) {}

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    MavKit& mav_;
};

} // namespace maxt
