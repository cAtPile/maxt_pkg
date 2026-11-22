#pragma once

#include <ros/ros.h>
#include <slicer_air_ground_msgs/V2Command.h>
#include <slicer_air_ground_msgs/V2State.h>
#include <uav_qr_mission/MissionResult.h>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace maxt {

class QrUgvDispatch {
public:
    explicit QrUgvDispatch(ros::NodeHandle nh);
    void begin(const std::string& mission, const std::string& target,
               const std::vector<std::string>& ids);
    void observe(const uav_qr_mission::MissionResult& result);

private:
    void onUgvState(const slicer_air_ground_msgs::V2State::ConstPtr& message);

    std::mutex mutex_;
    std::string mission_, target_;
    std::set<std::string> ids_, submitted_;
    double state_max_age_{2.0};
    double state_stamp_{0.0};
    ros::SteadyTime state_received_;
    ros::Publisher intent_;
    ros::Subscriber state_;
};

}  // namespace maxt
