#!/usr/bin/env python3
"""从 ugv_target.yaml 顺序读取绝对坐标，发送 UGV Action 目标。"""
from pathlib import Path

import actionlib
from actionlib_msgs.msg import GoalStatus
import rospkg
import rospy
import yaml
from maxt_pkg.msg import ugv_moverAction, ugv_moverFeedback, ugv_moverGoal


def main():
    rospy.init_node('ugv_controller')
    default = Path(rospkg.RosPack().get_path('maxt_pkg')) / 'config/ugv_target.yaml'
    with open(rospy.get_param('~config', str(default)), encoding='utf-8') as stream:
        targets = yaml.safe_load(stream)['targets']
    client = actionlib.SimpleActionClient(
        rospy.get_param('~action_name', '/ugv_mover'), ugv_moverAction)
    # 仅取消本控制台发送的目标。
    rospy.on_shutdown(client.cancel_goal)
    rospy.loginfo('等待 UGV Action 服务...')
    while not rospy.is_shutdown():
        if client.wait_for_server(rospy.Duration(0.5)):
            break

    states = {ugv_moverFeedback.WAITING: 'WAITING',
              ugv_moverFeedback.STARTING: 'STARTING',
              ugv_moverFeedback.MOVING: 'MOVING',
              ugv_moverFeedback.REACHED: 'REACHED'}

    def feedback(message):
        rospy.loginfo_throttle(1.0, 'UGV: %s', states[message.state])

    for index, target in enumerate(targets, start=1):
        if rospy.is_shutdown():
            return
        rospy.loginfo('发送目标 %d: (%s, %s)', index, target['x'], target['y'])
        client.send_goal(ugv_moverGoal(target_x=target['x'], target_y=target['y']),
                         feedback_cb=feedback)
        while not rospy.is_shutdown():
            if client.wait_for_result(rospy.Duration(0.2)):
                break
        if rospy.is_shutdown():
            return
        result = client.get_result()
        if client.get_state() != GoalStatus.SUCCEEDED or not result or not result.succeed:
            rospy.logerr('目标 %d 未完成，停止发送后续目标', index)
            return
        rospy.loginfo('目标 %d 已到达', index)
    rospy.loginfo('全部目标已完成')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
