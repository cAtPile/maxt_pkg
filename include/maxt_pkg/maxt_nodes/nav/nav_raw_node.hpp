#ifndef MAXT_NODES_NAV_RAW_HPP
#define MAXT_NODES_NAV_RAW_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

class NavRawNode : public MavActionNode {
public:
    NavRawNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav, const ros::NodeHandle& nh);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    BT::NodeStatus startNavigation();
    bool waiting_for_stop_{false};
    ros::NodeHandle nav_nh_;
    double tx_{0.0}, ty_{0.0}, tz_{0.0};
    double tolerance_{0.1};
    double timeout_{60.0};

    double T_{1.0};
    ros::Time traj_start_;
    ros::Time start_time_;
    ros::Time last_time_;
    ros::Time hold_start_;
    bool holding_{false};
    double hold_timeout_{3.0};

};

} // namespace maxt

#endif
