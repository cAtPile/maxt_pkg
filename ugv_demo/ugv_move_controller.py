#!/usr/bin/env python3
"""Send configured targets sequentially; cancel the current task on shutdown."""
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from maxt_pkg.msg import ugv_moverAction, ugv_moverGoal, ugv_moverFeedback


STATES = {ugv_moverFeedback.WAITING: 'WAITING',
          ugv_moverFeedback.STARTING: 'STARTING',
          ugv_moverFeedback.MOVING: 'MOVING',
          ugv_moverFeedback.REACHED: 'REACHED'}


def main():
    rospy.init_node('ugv_move_controller')
    client = actionlib.SimpleActionClient(
        rospy.get_param('~action_name', '/ugv_mover'), ugv_moverAction)
    rospy.on_shutdown(client.cancel_goal)
    wait = float(rospy.get_param('~server_timeout', 60.0))
    if not client.wait_for_server(rospy.Duration(wait)):
        rospy.logerr('UGV action server unavailable')
        return
    profile = rospy.get_param('~profile', 'point_3_3')
    targets = rospy.get_param('~profiles')[profile]
    last_state = None

    def feedback(msg):
        nonlocal last_state
        if msg.state != last_state:
            rospy.loginfo('UGV feedback: %s', STATES.get(msg.state, str(msg.state)))
            last_state = msg.state

    for target in targets:
        if rospy.is_shutdown():
            return
        last_state = None
        goal = ugv_moverGoal(target_x=target['x'], target_y=target['y'])
        rospy.loginfo('Sending target ENU (%.2f, %.2f)', goal.target_x, goal.target_y)
        client.send_goal(goal, feedback_cb=feedback)
        client.wait_for_result()
        result = client.get_result()
        if client.get_state() != GoalStatus.SUCCEEDED or result is None or not result.succeed:
            rospy.logerr('UGV mission stopped: %s', client.get_goal_status_text())
            return
    rospy.loginfo('All configured UGV targets reached')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
