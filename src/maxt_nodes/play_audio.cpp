#include <maxt_pkg/maxt_nodes/play_audio_node.hpp>

namespace maxt {

PlayAudioNode::PlayAudioNode(const std::string& name,
                           const BT::NodeConfiguration& config, MavKit& mav)
    : BT::SyncActionNode(name, config), mav_(mav) {}

BT::PortsList PlayAudioNode::providedPorts() {
    return {BT::InputPort<std::string>("filename", "WAV filename to play")};
}

BT::NodeStatus PlayAudioNode::tick() {
    std::string filename;
    if (!getInput("filename", filename)) {
        ROS_ERROR("PlayAudioNode: Missing filename input");
        return BT::NodeStatus::FAILURE;
    }
    mav_.playAudio(filename);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace maxt
