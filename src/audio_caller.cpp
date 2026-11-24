#include <maxt_pkg/audio_caller.hpp>
#include <maxt_pkg/PlayAudio.h>
#include <maxt_pkg/Say.h>
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
    bool speech{false};
    std::string pending;
};

AudioCaller::AudioCaller(const ros::NodeHandle& nh, const std::string& service_name)
    : state_(std::make_shared<State>()) {
    const auto state = state_;
    // The worker owns its ROS handle and state, never the AudioCaller object.
    std::thread([state, nh = ros::NodeHandle(nh), service_name]() mutable {
        try {
            auto client = nh.serviceClient<maxt_pkg::PlayAudio>(service_name, false);
            auto say_client = nh.serviceClient<maxt_pkg::Say>("/audio/say", false);
            std::unique_lock<std::mutex> lock(state->mutex);
            while (!state->stopped.load()) {
                // A timed wait also handles stop notifications racing with wait().
                state->wake.wait_for(lock, std::chrono::milliseconds(100), [&] {
                    return state->stopped.load() || !state->pending.empty();
                });
                if (state->stopped.load() || !ros::ok()) break;
                if (state->pending.empty()) continue;

                auto value = std::move(state->pending);
                const bool speech = state->speech;
                state->pending.clear();
                state->busy = true;
                lock.unlock();
                if (!state->stopped.load()) {
                    if (speech && say_client.exists()) {
                        maxt_pkg::Say request;
                        request.request.text = std::move(value);
                        say_client.call(request);
                    } else if (!speech && client.exists()) {
                        maxt_pkg::PlayAudio request;
                        request.request.filename = std::move(value);
                        client.call(request);
                    }
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

void AudioCaller::play(std::string filename, bool priority) {
    enqueue(std::move(filename), false, priority);
}

void AudioCaller::say(std::string text) {
    enqueue(std::move(text), true, false);
}

void AudioCaller::enqueue(std::string value, bool speech, bool priority) {
    if (value.empty() || state_->stopped.load()) return;
    std::unique_lock<std::mutex> lock(state_->mutex, std::defer_lock);
    if (priority) {
        lock.lock();  // The worker releases this mutex before every service call.
    } else if (!lock.try_lock() || state_->busy || !state_->pending.empty()) {
        return;
    }
    state_->pending = std::move(value);
    state_->speech = speech;
    state_->wake.notify_one();
}

}  // namespace maxt
