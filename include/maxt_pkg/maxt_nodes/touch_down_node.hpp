#ifndef MAXT_NODES_TOUCH_DOWN_HPP
#define MAXT_NODES_TOUCH_DOWN_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class TouchDownNode : public MavActionNode {
public:
    TouchDownNode(const std::string& name,
                  const BT::NodeConfiguration& config,
                  MavKit& mav);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    enum class Phase : uint8_t {
        INIT,
        DESCEND,
        DISARM,
        DONE
    } phase_{Phase::INIT};

    double descent_speed_{-0.5};
    double timeout_{60.0};
    double ground_timeout_{3.0};
    double height_tol_{0.3};
    double vel_tol_{0.2};

    double hold_x_{0.0};
    double hold_y_{0.0};
    double home_z_{0.0};

    ros::Time start_time_;
    ros::Time land_confirm_time_;
    bool land_confirming_{false};
    ros::Time disarm_request_time_;
};

}  // namespace maxt

#endif  // MAXT_NODES_TOUCH_DOWN_HPP
