#ifndef MAXT_NODES_YOLO_RECEIVER_HPP
#define MAXT_NODES_YOLO_RECEIVER_HPP

#include <maxt_pkg/maxt_nodes.hpp>
#include "maxt_pkg/StringStamped.h"

namespace maxt {

class YoloReceiverNode : public MavActionNode {
public:
    YoloReceiverNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    ros::ServiceClient toggle_yolo_detect_client;
    ros::Subscriber yolo_sub_;
    std::vector<std::string> yolo_data_;
    std::atomic<bool> received_{false};
    std::atomic<bool> yolo_detecting_{false};
    mutable std::mutex yolo_mutex_;
    ros::Time start_time_;

    void yoloCallback(const maxt_pkg::StringStamped::ConstPtr& msg);
};

} // namespace maxt

#endif
