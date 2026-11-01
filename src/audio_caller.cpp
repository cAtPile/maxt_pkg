#include <maxt_pkg/audio_caller.hpp>
#include <maxt_pkg/PlayAudio.h>
#include <ros/ros.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>

namespace maxt {

struct AudioCaller::State {
    std::atomic<bool> stopped{false};
    std::mutex mutex;
    std::condition_variable wake;
    bool busy{false};
    std::string pending;
};

AudioCaller::AudioCaller(const ros::NodeHandle& nh, const std::string& service_name)
    : state_(std::make_shared<State>()) {
    const auto state = state_;
    // The worker owns its ROS handle and state, never the AudioCaller object.
    std::thread([state, nh = ros::NodeHandle(nh), service_name]() mutable {
        try {
            auto client = nh.serviceClient<maxt_pkg::PlayAudio>(service_name, false);
            std::unique_lock<std::mutex> lock(state->mutex);
            while (!state->stopped.load()) {
                // A timed wait also handles stop notifications racing with wait().
                state->wake.wait_for(lock, std::chrono::milliseconds(100), [&] {
                    return state->stopped.load() || !state->pending.empty();
                });
                if (state->stopped.load() || !ros::ok()) break;
                if (state->pending.empty()) continue;

                maxt_pkg::PlayAudio request;
                request.request.filename = std::move(state->pending);
                state->pending.clear();
                state->busy = true;
                lock.unlock();
                // Both existence checking and the synchronous call stay off the main thread.
                // No waiting for availability, retries, or processing of response results.
                if (!state->stopped.load() && client.exists()) {
                    if (!state->stopped.load()) client.call(request);
                }
                lock.lock();
                state->busy = false;
            }
        } catch (const std::exception& error) {
            ROS_WARN("Audio worker stopped: %s", error.what());
        }
        state->stopped.store(true);
    }).detach();
}

AudioCaller::~AudioCaller() {
    // ROS1 synchronous calls have no general call timeout. Never join here:
    // an in-flight call may finish later using only the worker's shared state.
    state_->stopped.store(true);
    state_->wake.notify_one();
}

void AudioCaller::play(std::string filename) {
    if (filename.empty() || state_->stopped.load()) return;
    std::unique_lock<std::mutex> lock(state_->mutex, std::try_to_lock);
    if (!lock.owns_lock() || state_->busy || !state_->pending.empty()) return;
    state_->pending = std::move(filename);
    state_->wake.notify_one();
}

}  // namespace maxt
