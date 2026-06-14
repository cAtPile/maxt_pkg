#ifndef MAXT_NODES_TARGET_CHECK2_HPP
#define MAXT_NODES_TARGET_CHECK2_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class TargetCheck2Node : public BT::ConditionNode {
public:
    TargetCheck2Node(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

} // namespace maxt

#endif
