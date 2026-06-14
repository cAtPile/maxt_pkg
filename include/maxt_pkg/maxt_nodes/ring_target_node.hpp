#ifndef MAXT_NODES_RING_TARGET_HPP
#define MAXT_NODES_RING_TARGET_HPP

#include <maxt_pkg/maxt_nodes.hpp>
#include <geometry_msgs/PoseStamped.h>

namespace maxt {

class RingTargetNode : public MavActionNode {
public:
    RingTargetNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void ringCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    ros::Subscriber ring_sub_;
    std::atomic<bool> ring_detected_{false};
    std::atomic<bool> ring_locked_{false};
    double ring_center_x_{0.0};
    double ring_center_y_{0.0};
    int current_phase_{0};
    ros::Time start_time_;

    std::vector<double> x_samples_;
    std::vector<double> y_samples_;
    std::atomic<int> sample_count_{0};
    int skip_count_{0};
    static constexpr int MAX_SAMPLES = 20;
    static constexpr int SKIP_SAMPLES = 10;
    mutable std::mutex ring_mutex_;
};

} // namespace maxt

#endif
