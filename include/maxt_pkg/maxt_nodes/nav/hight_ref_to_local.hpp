#pragma once

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class HightRef2Local : public BT::SyncActionNode {
public:
    HightRef2Local(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

} // namespace maxt
