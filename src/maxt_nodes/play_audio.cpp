#include <maxt_pkg/maxt_nodes/play_audio_node.hpp>
#include <iomanip>
#include <locale>
#include <sstream>

namespace maxt {
namespace {

int audioLevel(const std::string& level) {
    using namespace ros::console::levels;
    if (level == "DEBUG") return Debug;
    if (level == "INFO") return Info;
    if (level == "WARN") return Warn;
    if (level == "ERROR") return Error;
    if (level == "FATAL") return Fatal;
    return -1;
}

}  // namespace

PlayAudioNode::PlayAudioNode(const std::string& name,
                             const BT::NodeConfiguration& config, MavKit& mav,
                             const ros::NodeHandle& nh)
    : BT::SyncActionNode(name, config), mav_(mav) {
    std::string minimum;
    nh.param<std::string>("qr_survey/audio_level", minimum, "INFO");
    minimum_level_ = audioLevel(minimum);
    if (minimum_level_ < 0) {
        ROS_WARN("Unknown qr_survey/audio_level '%s'; using INFO.", minimum.c_str());
        minimum_level_ = ros::console::levels::Info;
    }
}

BT::PortsList PlayAudioNode::providedPorts() {
    return {BT::InputPort<std::string>("filename", "WAV filename to play"),
            BT::InputPort<std::string>("level", "INFO", "Announcement severity"),
            BT::InputPort<bool>("priority", false, "Queue even if another notification is in flight"),
            BT::InputPort<int>("waypoint_number"),
            BT::InputPort<double>("x"), BT::InputPort<double>("y")};
}

BT::NodeStatus PlayAudioNode::tick() {
    std::string level;
    getInput("level", level);
    const int severity = audioLevel(level);
    if (severity < 0) ROS_WARN("Unknown announcement level '%s'.", level.c_str());
    if (severity < minimum_level_) return BT::NodeStatus::SUCCESS;

    std::string filename;
    if (getInput("filename", filename)) {
        bool priority = false;
        getInput("priority", priority);
        mav_.playAudio(filename, priority);
    } else {
        int waypoint;
        double x, y;
        if (getInput("waypoint_number", waypoint) && getInput("x", x) && getInput("y", y)) {
            std::ostringstream speech;
            speech.imbue(std::locale::classic());
            speech << "到达第" << waypoint << "个航点，坐标，"
                   << std::setprecision(6) << (x == 0 ? 0 : x) << "，" << (y == 0 ? 0 : y) << "。";
            mav_.sayAudio(speech.str());
        } else {
            ROS_WARN("PlayAudioNode: missing filename or waypoint inputs.");
        }
    }
    return BT::NodeStatus::SUCCESS;
}

}  // namespace maxt
