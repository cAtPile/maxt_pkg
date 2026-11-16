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
        WAIT_ARM,
        OFFBOARD,
        INIT,
        ASCEND,
        SETTLE
    } phase_{Phase::WAIT_ARM};

    double target_alt_{1.5};
    double ascent_speed_{0.5};
    double timeout_{30.0};

    double hold_yaw_{0.0};
    double aim_altitude_raw_{0.0};
    double altitude_tolerance_{0.1};
    double vertical_velocity_tolerance_{0.2};
    double settle_time_{0.4};
    geometry_msgs::PoseStamped goal_raw_;
    geometry_msgs::Point last_valid_position_;
    mavros_msgs::PositionTarget hold_target_;
    bool target_initialized_{false};
    ros::SteadyTime start_time_, stable_since_;

    void holdTarget();
    void stopClimb();
};

} // namespace maxt

#endif
