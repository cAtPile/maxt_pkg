#!/usr/bin/env python3
'''
先阅读SupremePrinciple.md

1. 创建 ugv_sim.py 一个简单的仿真小车的程序，无需使用px4，仅仅需要用mathplot仿真小车的运动与路径即可，保持轻量化
2. 创建 config/ugv_confg.yaml 来配置小车的速度
3. 根据 ugv_mover.action 使得 ugv_sim.py 可以根据action移动
4. 创建 ugv_controller.py 用来发布action控制小车移动
5. 使用 config/ugv_target.yaml 来设置ugv_controller发布的目标方向

'''
import math
import threading
import time
from pathlib import Path

import actionlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospkg
import rospy
import yaml
from maxt_pkg.msg import ugv_moverAction, ugv_moverFeedback, ugv_moverResult


class UgvSimulation:
    def __init__(self):
        default = Path(rospkg.RosPack().get_path('maxt_pkg')) / 'config/ugv_confg.yaml'
        with open(rospy.get_param('~config', str(default)), encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        self.speed = float(config['speed'])
        if not math.isfinite(self.speed) or self.speed <= 0:
            raise ValueError('speed 必须为有限正数（m/s）')
        self.lock = threading.Lock()
        self.x, self.y = 0.0, 0.0
        self.target = (0.0, 0.0)
        self.path = [(0.0, 0.0)]
        self.server = actionlib.SimpleActionServer(
            rospy.get_param('~action_name', '/ugv_mover'), ugv_moverAction,
            execute_cb=self.execute, auto_start=False)
        self.server.start()
        rospy.loginfo('UGV 仿真已就绪，速度 %.2f m/s', self.speed)

    def execute(self, goal):
        with self.lock:
            self.target = (goal.target_x, goal.target_y)
        self.server.publish_feedback(ugv_moverFeedback(state=ugv_moverFeedback.STARTING))
        previous = time.monotonic()
        # 墙钟推进仿真，不依赖 Gazebo /clock。
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.server.set_preempted(ugv_moverResult(succeed=False))
                return
            now = time.monotonic()
            dt, previous = now - previous, now
            with self.lock:
                dx, dy = goal.target_x - self.x, goal.target_y - self.y
                distance = math.hypot(dx, dy)
                step = min(self.speed * dt, distance)
                reached = distance <= step
                if reached:
                    self.x, self.y = goal.target_x, goal.target_y
                elif distance > 0:
                    self.x += dx / distance * step
                    self.y += dy / distance * step
                self.path.append((self.x, self.y))
            state = ugv_moverFeedback.REACHED if reached else ugv_moverFeedback.MOVING
            self.server.publish_feedback(ugv_moverFeedback(state=state))
            if reached:
                self.server.set_succeeded(ugv_moverResult(succeed=True))
                return
            time.sleep(1.0 / 30.0)
        self.server.set_aborted(ugv_moverResult(succeed=False))

    def show(self):
        # Matplotlib 只在主线程绘图，Action 线程通过锁共享位置。
        fig, ax = plt.subplots()
        ax.set(xlabel='X (m)', ylabel='Y (m)', title='UGV simulation')
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(True)
        trail, = ax.plot([], [], 'b-', label='Path')
        car, = ax.plot([], [], 'bo', label='UGV')
        target, = ax.plot([], [], 'rx', label='Target')
        ax.legend()

        def update(_):
            if rospy.is_shutdown():
                plt.close(fig)
                return
            with self.lock:
                xs, ys = zip(*self.path)
                trail.set_data(xs, ys)
                car.set_data([self.x], [self.y])
                target.set_data([self.target[0]], [self.target[1]])
            ax.relim()
            ax.autoscale_view()

        self.animation = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
        fig.canvas.mpl_connect('close_event', lambda _: rospy.signal_shutdown('窗口关闭'))
        plt.show()


if __name__ == '__main__':
    rospy.init_node('ugv_sim')
    UgvSimulation().show()
