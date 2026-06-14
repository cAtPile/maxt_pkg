#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def callback(msg):
    rospy.loginfo("Ring center: x=%.3f, y=%.3f, z=%.3f",
                  msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

rospy.init_node("ring_center_echo", anonymous=True)
rospy.Subscriber("/ring_center", PoseStamped, callback)
rospy.spin()
