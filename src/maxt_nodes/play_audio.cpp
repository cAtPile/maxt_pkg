#include <maxt_pkg/maxt_nodes/play_audio_node.hpp>

namespace maxt {

PlayAudioNode::PlayAudioNode(const std::string& name,
                           const BT::NodeConfiguration& config, MavKit& mav)
    : BT::SyncActionNode(name, config), mav_(mav) {}

BT::PortsList PlayAudioNode::providedPorts() {
    return {BT::InputPort<std::string>("filename", "WAV filename to play"),
            BT::InputPort<bool>("priority", false, "Queue even if another notification is in flight")};
}

BT::NodeStatus PlayAudioNode::tick() {
    std::string filename;
    if (!getInput("filename", filename)) {
        ROS_ERROR("PlayAudioNode: Missing filename input");
        return BT::NodeStatus::FAILURE;
    }
    bool priority = false;
    getInput("priority", priority);
    mav_.playAudio(filename, priority);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace maxt
