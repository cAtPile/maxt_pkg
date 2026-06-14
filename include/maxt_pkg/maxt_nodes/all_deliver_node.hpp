#ifndef MAXT_NODES_ALL_DELIVER_HPP
#define MAXT_NODES_ALL_DELIVER_HPP

#include <maxt_pkg/maxt_nodes.hpp>
#include <std_msgs/Empty.h>

namespace maxt {

class AllDeliverNode : public MavActionNode {
public:
    AllDeliverNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav,
                   ros::Publisher& all_open, ros::Publisher& all_close);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    ros::Publisher& all_open_pub_;
    ros::Publisher& all_close_pub_;

    double deliver_delay_{2.0};
    double connect_delay_{0.2};
    int delivery_phase_{0};
    ros::Time deliver_start_;
    ros::Time connect_start_;
};

} // namespace maxt

#endif
