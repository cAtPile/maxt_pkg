#!/usr/bin/env python3
"""Move the image panel in world Y using simulation time and Gazebo's ROS API."""
import math
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


def main():
    rospy.init_node('moving_target')
    model_name = rospy.get_param('~model_name', 'tracking_target')
    x = rospy.get_param('~x', 3.0)
    y = rospy.get_param('~y', 0.0)
    z = rospy.get_param('~z', 1.5)
    amplitude = rospy.get_param('~amplitude', 2.0)
    period = rospy.get_param('~period', 12.0)
    rate_hz = rospy.get_param('~rate', 30.0)
    if period <= 0 or rate_hz <= 0 or amplitude < 0:
        raise ValueError('period/rate must be positive; amplitude must be nonnegative')
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    state = ModelState(model_name=model_name, reference_frame='world')
    state.pose.orientation.w = 1.0
    state.pose.position.x = x
    state.pose.position.z = z
    rate = rospy.Rate(rate_hz, reset=True)
    start = None
    previous = None
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        # Wait for /clock; restart phase if Gazebo resets its simulation clock.
        if now == 0:
            rate.sleep()
            continue
        if start is None or (previous is not None and now < previous):
            start = now
        previous = now
        phase = 2.0 * math.pi * (now - start) / period
        state.pose.position.y = y + amplitude * math.sin(phase)
        try:
            result = set_state(state)
            if not result.success:
                rospy.logwarn_throttle(5.0, result.status_message)
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(5.0, str(error))
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
