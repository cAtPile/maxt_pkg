#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty, Int32


class FakeDeliver:
    def __init__(self):
        self.doors = {
            "front_left":  False,
            "front_right": False,
            "back_right":  False,
            "back_left":   False,
        }

        rospy.Subscriber("/servo/front_left", Int32, self._fl_angle)
        rospy.Subscriber("/servo/front_left/open", Empty, self._fl_open)
        rospy.Subscriber("/servo/front_left/close", Empty, self._fl_close)

        rospy.Subscriber("/servo/front_right", Int32, self._fr_angle)
        rospy.Subscriber("/servo/front_right/open", Empty, self._fr_open)
        rospy.Subscriber("/servo/front_right/close", Empty, self._fr_close)

        rospy.Subscriber("/servo/back_right", Int32, self._br_angle)
        rospy.Subscriber("/servo/back_right/open", Empty, self._br_open)
        rospy.Subscriber("/servo/back_right/close", Empty, self._br_close)

        rospy.Subscriber("/servo/back_left", Int32, self._bl_angle)
        rospy.Subscriber("/servo/back_left/open", Empty, self._bl_open)
        rospy.Subscriber("/servo/back_left/close", Empty, self._bl_close)

        rospy.Subscriber("/servo/all/open", Empty, self._all_open)
        rospy.Subscriber("/servo/all/close", Empty, self._all_close)

        rospy.loginfo("FakeDeliver ready, listening on /servo/*")

    def _fl_angle(self, msg):
        rospy.loginfo("[Fake] front_left angle -> %d", msg.data)

    def _fl_open(self, _):
        self.doors["front_left"] = True
        rospy.loginfo("[Fake] front_left OPEN")

    def _fl_close(self, _):
        self.doors["front_left"] = False
        rospy.loginfo("[Fake] front_left CLOSED")

    def _fr_angle(self, msg):
        rospy.loginfo("[Fake] front_right angle -> %d", msg.data)

    def _fr_open(self, _):
        self.doors["front_right"] = True
        rospy.loginfo("[Fake] front_right OPEN")

    def _fr_close(self, _):
        self.doors["front_right"] = False
        rospy.loginfo("[Fake] front_right CLOSED")

    def _br_angle(self, msg):
        rospy.loginfo("[Fake] back_right angle -> %d", msg.data)

    def _br_open(self, _):
        self.doors["back_right"] = True
        rospy.loginfo("[Fake] back_right OPEN")

    def _br_close(self, _):
        self.doors["back_right"] = False
        rospy.loginfo("[Fake] back_right CLOSED")

    def _bl_angle(self,msg):
        rospy.loginfo("[Fake] back_left angle -> %d", msg.data)

    def _bl_open(self,_):
        self.doors["back_left"] = True
        rospy.loginfo("[Fake] back_left OPEN")

    def _bl_close(self, _):
        self.doors["back_left"] = False
        rospy.loginfo("[Fake] back_left CLOSED")


    def _all_open(self, _):
        for k in self.doors:
            self.doors[k] = True
        rospy.loginfo("[Fake] ALL doors OPEN: %s", self.doors)

    def _all_close(self, _):
        for k in self.doors:
            self.doors[k] = False
        rospy.loginfo("[Fake] ALL doors CLOSED: %s", self.doors)


rospy.init_node("fake_deliver")
FakeDeliver()
rospy.spin()
