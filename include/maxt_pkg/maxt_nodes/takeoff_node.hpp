#ifndef MAXT_NODES_TAKEOFF_HPP
#define MAXT_NODES_TAKEOFF_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class TakeoffNode : public MavActionNode {
public:
    TakeoffNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    enum class Phase : uint8_t {
        ARM,
        OFFBOARD,
        INIT,
        ASCEND,
        DONE
    } phase_{Phase::ARM};

    double target_alt_{1.5};
    double ascent_speed_{0.5};
    double timeout_{20.0};

    double hold_x_{0.0}, hold_y_{0.0};
    double hold_yaw_{0.0};
    double target_z_{0.0};
    ros::Time start_time_;
};

} // namespace maxt

#endif
