#ifndef MAXT_NODES_DELIVER_HPP
#define MAXT_NODES_DELIVER_HPP

#include <maxt_pkg/maxt_nodes.hpp>
#include <std_msgs/Empty.h>

namespace maxt {

class DeliverNode : public MavActionNode {
public:
    DeliverNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav,
                ros::Publisher& fl_open, ros::Publisher& fl_close,
                ros::Publisher& fr_open, ros::Publisher& fr_close,
                ros::Publisher& bl_open, ros::Publisher& bl_close,
                ros::Publisher& br_open, ros::Publisher& br_close);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    static const std::vector<std::string> SERVO_NAMES;

    ros::Publisher* open_pubs_[4];
    ros::Publisher* close_pubs_[4];

    int servo_idx_a_{-1};
    int servo_idx_b_{-1};
    int chosen_servo_idx_{-1};
    double deliver_delay_{2.0};
    double connect_delay_{0.2};
    int delivery_phase_{0};
    ros::Time deliver_start_;
    ros::Time connect_start_;
};

} // namespace maxt

#endif
