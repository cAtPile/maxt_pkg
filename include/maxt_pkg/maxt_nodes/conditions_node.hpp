#ifndef MAXT_NODES_CONDITIONS_HPP
#define MAXT_NODES_CONDITIONS_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class CheckLandLeftNode : public BT::ConditionNode {
public:
    CheckLandLeftNode(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

class CheckLandRightNode : public BT::ConditionNode {
public:
    CheckLandRightNode(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

} // namespace maxt

#endif
