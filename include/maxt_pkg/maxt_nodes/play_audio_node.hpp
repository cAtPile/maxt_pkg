#ifndef MAXT_NODES_PLAY_AUDIO_HPP
#define MAXT_NODES_PLAY_AUDIO_HPP

#include <maxt_pkg/maxt_nodes.hpp>

namespace maxt {

// 每次执行只提交一次文件名，音频结果不影响行为树任务。
class PlayAudioNode : public BT::SyncActionNode {
public:
    PlayAudioNode(const std::string& name, const BT::NodeConfiguration& config,
                  MavKit& mav);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    MavKit& mav_;
};

}  // namespace maxt

#endif
