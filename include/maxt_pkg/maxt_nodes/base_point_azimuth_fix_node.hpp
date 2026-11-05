#ifndef MAXT_BASE_POINT_AZIMUTH_FIX_NODE_HPP
#define MAXT_BASE_POINT_AZIMUTH_FIX_NODE_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

// 一次 tick 记录原始基点，通过输出端口写入黑板。
class BasePointAzimuthFixNode : public BT::SyncActionNode {
public:
    BasePointAzimuthFixNode(const std::string& name, const BT::NodeConfiguration& config,
                            MavKit& mav);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    MavKit& mav_;
};

} // namespace maxt
#endif
