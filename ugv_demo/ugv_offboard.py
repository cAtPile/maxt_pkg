#!/usr/bin/env python3
"""ROS1/MAVROS PX4 rover: ENU goal tracking or timed velocity, then stop."""
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode


class RoverDemo:
    def __init__(self):
        self.speed = float(rospy.get_param('~speed', 0.5))
        self.duration = float(rospy.get_param('~duration', 5.0))
        self.heading = math.radians(float(rospy.get_param('~heading_deg', 0.0)))
        self.timeout = float(rospy.get_param('~startup_timeout', 60.0))
        self.use_goal = rospy.get_param('~use_goal', True)
        self.goal_x = float(rospy.get_param('~goal_x', 3.0))
        self.goal_y = float(rospy.get_param('~goal_y', 3.0))
        self.tolerance = float(rospy.get_param('~tolerance', 0.2))
        self.goal_timeout = float(rospy.get_param('~goal_timeout', 120.0))
        self.legacy_rover = rospy.get_param('~legacy_rover_velocity', True)
        if not all(math.isfinite(v) for v in
                   (self.speed, self.duration, self.heading, self.timeout,
                    self.goal_x, self.goal_y, self.tolerance, self.goal_timeout)):
            raise ValueError('Parameters must be finite')
        if not 0 <= self.speed <= 2 or self.duration < 0 or self.timeout <= 2:
            raise ValueError('Require 0 <= speed <= 2, duration >= 0, startup_timeout > 2')
        if self.tolerance <= 0 or self.goal_timeout <= 0 or (self.use_goal and self.speed <= 0):
            raise ValueError('Goal mode requires positive speed, tolerance and goal_timeout')
        ns = rospy.get_param('~mavros_ns', '/mavros').rstrip('/')
        self.lock = threading.Lock()
        self.state = State()
        self.state_time = 0.0
        self.drive_until = 0.0
        self.stopped = False
        self.pose = None
        self.pose_time = 0.0
        self.yaw = 0.0
        self.pub = rospy.Publisher(ns + '/setpoint_raw/local', PositionTarget, queue_size=1)
        self.sub = rospy.Subscriber(ns + '/state', State, self.on_state)
        self.pose_sub = rospy.Subscriber(ns + '/local_position/pose', PoseStamped, self.on_pose)
        self.mode = rospy.ServiceProxy(ns + '/set_mode', SetMode)
        self.arm = rospy.ServiceProxy(ns + '/cmd/arming', CommandBool)
        # A separate timer keeps streaming while service requests wait for replies.
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish)
        rospy.on_shutdown(self.shutdown)

    def on_state(self, msg):
        with self.lock:
            self.state = msg
            self.state_time = time.monotonic()

    def on_pose(self, msg):
        q = msg.pose.orientation
        if not all(math.isfinite(v) for v in
                   (msg.pose.position.x, msg.pose.position.y, q.x, q.y, q.z, q.w)):
            return
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-6:
            return
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        with self.lock:
            self.pose = msg.pose.position
            self.yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
            self.pose_time = time.monotonic()

    def publish(self, _event):
        now = time.monotonic()
        vx = vy = 0.0
        with self.lock:
            moving = (not self.stopped and now < self.drive_until
                      and now - self.state_time < 2.0 and self.state.connected
                      and self.state.armed and self.state.mode == 'OFFBOARD')
            if moving and self.use_goal:
                if self.pose is None or now - self.pose_time > 1.0:
                    rospy.logerr('Position feedback stale; stopping')
                    moving = False
                else:
                    dx, dy = self.goal_x - self.pose.x, self.goal_y - self.pose.y
                    distance = math.hypot(dx, dy)
                    if distance <= self.tolerance:
                        rospy.loginfo('Goal reached: (%.3f, %.3f), error %.3f m',
                                      self.pose.x, self.pose.y, distance)
                        moving = False
                    else:
                        speed = min(self.speed, 0.6 * distance)
                        vx, vy = speed * dx / distance, speed * dy / distance
                        rospy.loginfo_throttle(2.0, 'Goal error: %.3f m', distance)
            elif moving:
                vx = self.speed * math.cos(self.heading)
                vy = self.speed * math.sin(self.heading)
            if moving and self.legacy_rover:
                if self.pose is None or now - self.pose_time > 1.0:
                    rospy.logerr('Heading feedback stale; stopping')
                    moving = False
                else:
                    # Local PX4 RoverPositionControl::control_velocity uses the
                    # NED setpoint directly as BODY FRD, without inverse yaw.
                    # Supply the desired body vector through MAVROS ENU->NED:
                    # NED.x = ROS.y = forward; NED.y = ROS.x = right.
                    target_heading = math.atan2(vy, vx)
                    error = self.yaw - target_heading
                    speed = math.hypot(vx, vy)
                    vx = speed * math.sin(error)
                    vy = speed * math.cos(error)
            if self.drive_until and not moving:
                self.stopped = True
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # MAVROS converts ENU to NED. Legacy compensation above intentionally
        # encodes a body steering vector for this firmware's rover controller.
        msg.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY
                         | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX
                         | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                         | PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
        if moving:
            msg.velocity.x, msg.velocity.y = vx, vy
        self.pub.publish(msg)

    def run(self):
        deadline = time.monotonic() + self.timeout
        ready_since = None
        last_request = 0.0
        rospy.loginfo('Waiting for MAVROS; streaming zero velocity at 20 Hz')
        while not rospy.is_shutdown():
            now = time.monotonic()
            if now > deadline:
                rospy.logerr('Startup timeout: check PX4 rover, estimator and MAVROS connection')
                return
            with self.lock:
                state = self.state
                connected = state.connected and now - self.state_time < 2.0
                if self.use_goal or self.legacy_rover:
                    connected = connected and self.pose is not None and now - self.pose_time < 1.0
            if not connected:
                ready_since = None
            elif ready_since is None:
                ready_since = now
            elif now - ready_since >= 2.0:
                if state.mode == 'OFFBOARD' and state.armed:
                    with self.lock:
                        self.drive_until = now + (self.goal_timeout if self.use_goal else self.duration)
                    if self.use_goal:
                        rospy.loginfo('Navigating to ENU (%.2f, %.2f), tolerance %.2f m',
                                      self.goal_x, self.goal_y, self.tolerance)
                    else:
                        rospy.loginfo('Driving %.2f m/s for %.2f s (ENU heading %.1f deg)',
                                      self.speed, self.duration, math.degrees(self.heading))
                    break
                if now - last_request >= 2.0:
                    last_request = now
                    try:
                        if state.mode != 'OFFBOARD':
                            if not self.mode(custom_mode='OFFBOARD').mode_sent:
                                rospy.logwarn('OFFBOARD request rejected')
                        elif not self.arm(value=True).success:
                            rospy.logwarn('Arming rejected; inspect PX4 preflight checks')
                    except rospy.ServiceException as exc:
                        rospy.logwarn('PX4 service unavailable: %s', exc)
            rospy.rostime.wallsleep(0.1)
        while not rospy.is_shutdown():
            with self.lock:
                stopped = self.stopped
            if stopped:
                rospy.loginfo('Motion ended or control lost; holding zero velocity. Restart to rerun.')
                rospy.spin()
                return
            rospy.rostime.wallsleep(0.1)

    def shutdown(self):
        with self.lock:
            self.stopped = True
        self.timer.shutdown()
        self.publish(None)


if __name__ == '__main__':
    rospy.init_node('ugv_offboard_demo')
    try:
        RoverDemo().run()
    except (ValueError, rospy.ROSInterruptException) as exc:
        rospy.logerr('%s', exc)
