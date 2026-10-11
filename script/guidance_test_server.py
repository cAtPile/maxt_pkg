#!/usr/bin/env python3
"""收到 test 目标后发布 u 方向固定误差，10 秒后停止反馈。"""

import actionlib
import rospy

from maxt_pkg.msg import GuidanceAction, GuidanceFeedback, GuidanceResult


class GuidanceTestServer:
    def __init__(self):
        self.error_u = rospy.get_param("~error_u", 20.0)
        self.duration = rospy.get_param("~duration", 10.0)
        self.rate = rospy.get_param("~rate", 20.0)
        if self.duration <= 0.0 or self.rate <= 0.0:
            raise ValueError("duration 和 rate 必须大于 0")

        self.server = actionlib.SimpleActionServer(
            "/guidance",
            GuidanceAction,
            execute_cb=self.execute,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("Guidance 测试服务已启动：/guidance")

    def execute(self, goal):
        if goal.target_id != "test":
            self.server.set_aborted(GuidanceResult(message="FAIL: target_id must be test"))
            return

        rospy.loginfo("收到 test：发布 error_u=%s，持续 %s 秒", self.error_u, self.duration)
        start = rospy.Time.now()
        rate = rospy.Rate(self.rate)
        stopped = False

        while not rospy.is_shutdown():
            # Guidance 达到容差或超时时会取消 goal，结束本次反馈。
            if self.server.is_preempt_requested():
                result = GuidanceResult(message="PREEMPTED")
                self.server.set_preempted(result)
                rospy.loginfo("goal 已取消：%s", goal.target_id)
                return

            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed < self.duration:
                feedback = GuidanceFeedback(
                    target_visible=True,
                    error_u=self.error_u,
                    error_v=0.0,
                )
                self.server.publish_feedback(feedback)
            elif not stopped:
                rospy.loginfo("已停止反馈，保持 Action 活跃，等待 Guidance 取消")
                stopped = True

            # 不发送终态，专门测试反馈过期及客户端整体超时。
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("guidance_test_server")
    server = GuidanceTestServer()
    rospy.spin()
