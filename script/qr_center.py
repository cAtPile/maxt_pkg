#!/usr/bin/env python3
"""持续识别白名单二维码，提供视觉伺服反馈，并在对准后异步上报。"""

from collections import deque
import threading
from uuid import uuid4

import actionlib
import cv2  # Load OpenCV before its ROS bridge in the onboard Python environment.
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus
from std_msgs.msg import String
import zxingcpp

from maxt_pkg.msg import GuidanceAction, GuidanceFeedback, GuidanceResult
from slicer_air_ground_msgs.msg import V2Command


FRAME_MAX_AGE = 0.3
REVISIT_MISSING_TIMEOUT = 3.0
POSITION_MAX_AGE = 2.0


class QrCenter:
    def __init__(self):
        self.target_id = rospy.get_param("~target_id")
        report_ids = rospy.get_param("~report_id")
        if not isinstance(report_ids, list):
            raise ValueError("report_id 必须是列表")
        ids = [self.target_id, *report_ids]
        if any(not isinstance(code, str) or not code.strip() for code in ids) or len(set(ids)) != len(ids):
            raise ValueError("target_id 和 report_id 必须是非空且互不重复的二维码原文")
        self.allowed_ids = set(ids)
        self.publish_order = deque(ids)
        self.task_id = f"qr-traverse-{uuid4().hex}"
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None
        self.fix = None
        self.published = set()

        self.content_pub = rospy.Publisher("~content", String, queue_size=1)
        self.intent_pub = rospy.Publisher("/air_ground/v2/uav_intent", V2Command, queue_size=8)
        self.subscribers = [
            rospy.Subscriber("~aligned", String, self.on_aligned, queue_size=8),
            rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.on_fix, queue_size=1),
        ]
        if rospy.get_param("~run_guidance", False):
            self.server = actionlib.SimpleActionServer(
                "/guidance", GuidanceAction, execute_cb=self.execute_guidance, auto_start=False)
            self.server.start()

        topic = rospy.get_param("~image_topic")
        self.subscribers.append(rospy.Subscriber(
            topic, Image, self.on_image, queue_size=1, buff_size=2**24))
        rospy.loginfo("二维码识别已启动，目标码 %s，上报码 %s，图像 %s",
                      self.target_id, report_ids, topic)

    def on_image(self, msg):
        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(2, "二维码图像转换失败：%s", error)
            return

        errors = {}
        for barcode in zxingcpp.read_barcodes(gray, formats=zxingcpp.BarcodeFormat.QRCode):
            if not barcode.valid or barcode.text not in self.allowed_ids:
                continue
            pos = barcode.position
            corners = (pos.top_left, pos.top_right, pos.bottom_right, pos.bottom_left)
            if not all(0 <= p.x < msg.width and 0 <= p.y < msg.height for p in corners):
                continue
            u = sum(p.x for p in corners) / 4
            v = sum(p.y for p in corners) / 4
            errors[barcode.text] = (u - (msg.width - 1) / 2, v - (msg.height - 1) / 2)

        with self.lock:
            self.frame = (msg.header.stamp, errors)

        # 同一画面有多个白名单码时轮流发布，保持 TargetCheck 的 String 接口。
        code = next((code for code in self.publish_order if code in errors), "")
        if code:
            self.publish_order.remove(code)
            self.publish_order.append(code)
        self.content_pub.publish(String(data=code))

    def execute_guidance(self, goal):
        code = goal.target_id
        if code not in self.allowed_ids:
            self.server.set_aborted(GuidanceResult(message="QR_NOT_ALLOWED"))
            return

        started = rospy.Time.now()
        missing_since = None
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.server.set_preempted(GuidanceResult(message="PREEMPTED"))
                return
            now = rospy.Time.now()
            with self.lock:
                frame = self.frame
            if frame is None or frame[0] < started or not 0 <= (now - frame[0]).to_sec() <= FRAME_MAX_AGE:
                missing_since = None
            else:
                error = frame[1].get(code)
                if error is not None:
                    missing_since = None
                    self.server.publish_feedback(GuidanceFeedback(
                        target_visible=True, error_u=error[0], error_v=error[1]))
                else:
                    self.server.publish_feedback(GuidanceFeedback(target_visible=False))
                    if missing_since is None:
                        missing_since = now
                    elif (now - missing_since).to_sec() >= REVISIT_MISSING_TIMEOUT:
                        rospy.logwarn("回访未找到二维码 %s，跳过该目标", code)
                        self.server.set_aborted(GuidanceResult(message="QR_NOT_VISIBLE"))
                        return
            rate.sleep()

    def on_fix(self, fix):
        with self.lock:
            self.fix = fix

    def on_aligned(self, msg):
        code = msg.data
        now = rospy.Time.now()
        with self.lock:
            if code not in self.allowed_ids or code in self.published:
                return
            fix = self.fix
            if (fix is None or fix.status.status < NavSatStatus.STATUS_FIX or
                    fix.header.stamp.is_zero() or not 0 <= (now - fix.header.stamp).to_sec() <= POSITION_MAX_AGE or
                    not abs(fix.latitude) <= 90 or not abs(fix.longitude) <= 180):
                rospy.logwarn("二维码 %s 已对准，但没有新鲜有效的 GPS，跳过上报", code)
                return
            # 下视对准时的无人机经纬度作为标靶水平位置的近似值，不填地面海拔。
            command = V2Command(task_id=self.task_id)
            if code == self.target_id:
                command.op = "SET_TARGET"
                command.has_target = True
                command.target.lat = fix.latitude
                command.target.lon = fix.longitude
            else:
                command.op = "REPORT_QR"
                command.qr.qr_id = code
                command.qr.lat = fix.latitude
                command.qr.lon = fix.longitude
                command.qr.measured_at = fix.header.stamp.to_sec()
                command.qr.revision = 1
            self.intent_pub.publish(command)
            self.published.add(code)
        rospy.loginfo("二维码 %s 已发布上报，操作 %s，任务 %s",
                      code, command.op, command.task_id)


if __name__ == "__main__":
    rospy.init_node("qr_center")
    try:
        node = QrCenter()
    except (KeyError, ValueError) as error:
        rospy.logfatal("二维码配置错误：%s", error)
        raise SystemExit(1)
    rospy.spin()
