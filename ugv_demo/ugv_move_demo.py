#!/usr/bin/env python3
"""Action-controlled rover; reuse the standalone demo's 20 Hz control loop."""
import os
import sys
import time

import actionlib
import rospy
from maxt_pkg.msg import ugv_moverAction, ugv_moverFeedback, ugv_moverResult

# catkin's devel relay executes this source with its original __file__,
# but keeps sys.path at devel/lib. Import the sibling source, not its relay.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ugv_offboard import RoverDemo


class UgvMoveDemo(RoverDemo):
    def __init__(self):
        super().__init__()
        self.use_goal = True
        if self.speed <= 0:
            raise ValueError('Action goal tracking requires positive speed')
        self.server = actionlib.SimpleActionServer(
            rospy.get_param('~action_name', '/ugv_mover'), ugv_moverAction,
            execute_cb=self.execute, auto_start=False)
        self.server.start()
        rospy.loginfo('UGV action server ready; waiting for targets')

    def feedback(self, state):
        self.server.publish_feedback(ugv_moverFeedback(state=state))

    def finish(self, status, reason):
        # Stop streaming movement before publishing any terminal action state.
        with self.lock:
            self.stopped = True
            self.drive_until = 0.0
        self.publish(None)
        result = ugv_moverResult(succeed=status == 'succeeded')
        getattr(self.server, 'set_' + status)(result, reason)
        rospy.loginfo('UGV task %s: %s', status, reason)

    def execute(self, goal):
        with self.lock:
            self.goal_x, self.goal_y = goal.target_x, goal.target_y
            self.stopped = False
            self.drive_until = 0.0
        self.feedback(ugv_moverFeedback.WAITING)
        deadline = time.monotonic() + self.timeout
        ready_since = None
        last_request = 0.0
        moving = False
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.finish('preempted', 'Cancelled or replaced by a new target')
                return
            now = time.monotonic()
            with self.lock:
                state = self.state
                ready = (state.connected and now - self.state_time < 2.0
                         and self.pose is not None and now - self.pose_time < 1.0)
                controlled = ready and state.armed and state.mode == 'OFFBOARD'
                reached = (controlled and
                           ((self.goal_x - self.pose.x)**2 +
                            (self.goal_y - self.pose.y)**2)**0.5 <= self.tolerance)
                stopped = self.stopped
            if moving:
                if reached:
                    self.feedback(ugv_moverFeedback.REACHED)
                    self.finish('succeeded', 'Target reached within tolerance')
                    return
                if stopped or not controlled or now >= deadline:
                    self.finish('aborted', 'Motion timeout or vehicle/pose/control lost')
                    return
                self.feedback(ugv_moverFeedback.MOVING)
            else:
                if now >= deadline:
                    self.finish('aborted', 'Startup timeout waiting for PX4 readiness')
                    return
                self.feedback(ugv_moverFeedback.STARTING if ready
                              else ugv_moverFeedback.WAITING)
                if not ready:
                    ready_since = None
                elif ready_since is None:
                    ready_since = now
                elif now - ready_since >= 2.0:
                    if controlled:
                        deadline = now + self.goal_timeout
                        with self.lock:
                            self.drive_until = deadline
                        moving = True
                        rospy.loginfo('Action target ENU (%.2f, %.2f)', goal.target_x, goal.target_y)
                    elif now - last_request >= 2.0:
                        last_request = now
                        try:
                            if state.mode != 'OFFBOARD':
                                if not self.mode(custom_mode='OFFBOARD').mode_sent:
                                    rospy.logwarn('OFFBOARD request rejected')
                            elif not self.arm(value=True).success:
                                rospy.logwarn('Arming request rejected')
                        except rospy.ServiceException as exc:
                            rospy.logwarn('PX4 service unavailable: %s', exc)
            rospy.rostime.wallsleep(0.1)
        self.finish('aborted', 'ROS shutdown')


if __name__ == '__main__':
    rospy.init_node('ugv_move_demo')
    try:
        UgvMoveDemo()
        rospy.spin()
    except (ValueError, rospy.ROSInterruptException) as exc:
        rospy.logerr('%s', exc)
