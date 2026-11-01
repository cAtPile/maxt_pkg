#!/usr/bin/env python3
"""模拟 /audio/play：立即响应，并在指定时长内拒绝重复播放。"""

import math
import threading
import time

import rospy

from maxt_pkg.srv import PlayAudio, PlayAudioResponse


class FakeAudio:
    def __init__(self):
        self.duration = float(rospy.get_param("~duration", 3.0))
        if not math.isfinite(self.duration) or self.duration < 0:
            raise ValueError("duration error")
        self.lock = threading.Lock()
        self.busy_until = 0.0
        self.service = rospy.Service("/audio/play", PlayAudio, self.play)
        rospy.loginfo("Fake audio service ready: /audio/play (duration=%.2f s)", self.duration)

    def play(self, request):
        with self.lock:
            now = time.monotonic()
            if now < self.busy_until:
                return PlayAudioResponse(False, "Audio device busy")

            name = request.filename
            if (len(name) <= 4 or not name.endswith(".wav")
                    or any(value in name for value in ("/", "\\", "..", "\0"))):
                return PlayAudioResponse(
                    False, "Invalid filename: use a plain filename ending in .wav"
                )

            # 不检查真实文件，也不等待播放结束；墙钟时间不依赖仿真 /clock。
            self.busy_until = now + self.duration
            rospy.loginfo("Simulated playback: %s (duration=%.2f s)", name, self.duration)
            return PlayAudioResponse(True, "Playback started")


if __name__ == "__main__":
    rospy.init_node("fake_audio")
    server = FakeAudio()
    rospy.spin()
