#ifndef MAXT_NODES_QR_DETECT_HPP
#define MAXT_NODES_QR_DETECT_HPP

#include <maxt_pkg/maxt_nodes.hpp>
#include "maxt_pkg/StringStamped.h"

namespace maxt {

class QRDetectNode : public MavActionNode {
public:
    QRDetectNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    ros::Subscriber qr_sub_;
    std::vector<std::string> qr_data_;
    std::atomic<bool> received_{false};
    mutable std::mutex qr_mutex_;
    ros::Time start_time_;
    double timeout_;

    void qrCallback(const maxt_pkg::StringStamped::ConstPtr& msg);
};

} // namespace maxt

#endif
