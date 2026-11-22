#ifndef MAXT_PKG_AUDIO_CALLER_HPP
#define MAXT_PKG_AUDIO_CALLER_HPP

#include <memory>
#include <string>

namespace ros { class NodeHandle; }

namespace maxt {

// Best-effort audio notifications. No ROS operations run on the calling thread.
// Keep one instance for the application's lifetime. Results are intentionally ignored.
class AudioCaller {
public:
    explicit AudioCaller(const ros::NodeHandle& nh,
                         const std::string& service_name = "/audio/play");
    ~AudioCaller();

    AudioCaller(const AudioCaller&) = delete;
    AudioCaller& operator=(const AudioCaller&) = delete;
    AudioCaller(AudioCaller&&) = delete;
    AudioCaller& operator=(AudioCaller&&) = delete;

    // Priority requests replace any pending notification and run after an in-flight call.
    void play(std::string filename, bool priority = false);

private:
    struct State;
    std::shared_ptr<State> state_;
};

}  // namespace maxt

#endif
