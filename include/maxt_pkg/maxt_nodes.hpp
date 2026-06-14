#ifndef MAXT_NODES_HPP
#define MAXT_NODES_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <maxt_pkg/maxt_mav.hpp>

namespace maxt {

/**
 * @brief 基础动作节点类，提供对 MavKit 的访问权限
 */
class MavActionNode : public BT::StatefulActionNode {
public:
    MavActionNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
        : BT::StatefulActionNode(name, config), mav_(mav) {}

    MavActionNode() = delete;

protected:
    MavKit& mav_;
};

} // namespace maxt

#endif // MAXT_NODES_HPP
