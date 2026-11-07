#!/usr/bin/env python3
"""接收二维码经纬度消息，打印完整内容并回复接收确认。"""

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool


class FakeReceiver:
    def __init__(self):
        self.ack_pub = rospy.Publisher("QRCode_pose/ack", Bool, queue_size=10)
        self.pose_sub = rospy.Subscriber(
            "QRCode_pose/Global_pose", NavSatFix, self.receive_pose, queue_size=10
        )
        rospy.loginfo("Fake receiver ready: QRCode_pose/Global_pose -> QRCode_pose/ack")

    def receive_pose(self, message):
        rospy.loginfo("已收到经纬度信息：\n%s", message)
        self.ack_pub.publish(Bool(data=True))


if __name__ == "__main__":
    rospy.init_node("fake_reciver")
    receiver = FakeReceiver()
    rospy.spin()
