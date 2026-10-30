#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import PoseStamped

class XYTrajectoryMonitor:
    def __init__(self):
        rospy.init_node('xytraj_monitor', anonymous=True)
        
        # 获取参数
        self.pose_topic = rospy.get_param('~pose_topic', '/mavros/local_position/pose')
        self.x_range = rospy.get_param('~x_range', [-1.0, 8.0])
        self.y_range = rospy.get_param('~y_range', [-3.0, 3.0])
        self.fig_size = rospy.get_param('~fig_size', (10, 8))
        self.point_size = rospy.get_param('~point_size', 3)
        
        # 不限制历史点数；订阅回调与绘图线程通过锁同步。
        self.data_lock = threading.Lock()
        self.x_data = []
        self.y_data = []
        self.timestamps = []
        
        # 创建图形
        self.fig, self.ax = plt.subplots(figsize=self.fig_size)
        self.ax.set_xlim(self.x_range)
        self.ax.set_ylim(self.y_range)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('XY Trajectory Monitor')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        
        # 初始化散点图
        self.scatter = self.ax.scatter([], [], c=[], cmap='hot', s=self.point_size, alpha=0.8, vmin=0, vmax=1)
        
        self.start_marker, = self.ax.plot([], [], 'go', markersize=10, label='Start')
        self.current_marker, = self.ax.plot([], [], 'r*', markersize=12, label='Current')
        self.ax.legend(loc='upper right')

        # 在图形和数据初始化完成后开始接收消息。
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)

        rospy.loginfo("XY Trajectory Monitor started!")
        rospy.loginfo("Subscribing to: %s", self.pose_topic)
        
    def pose_callback(self, msg):
        """处理位置消息"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        t = msg.header.stamp.to_sec()
        
        if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(t)):
            return

        with self.data_lock:
            self.x_data.append(x)
            self.y_data.append(y)
            self.timestamps.append(t)

    def update_plot(self, frame):
        """显示全部历史点，并让坐标范围随航迹扩展。"""
        with self.data_lock:
            if not self.x_data:
                return self.scatter,
            # 获取一致的快照，避免回调追加数据时数组长度不一致。
            points = np.column_stack((self.x_data, self.y_data))
            timestamps = np.asarray(self.timestamps)

        t_min = timestamps.min()
        t_max = timestamps.max()
        if t_max > t_min:
            time_norm = (timestamps - t_min) / (t_max - t_min)
        else:
            time_norm = np.zeros(len(timestamps))

        self.scatter.set_offsets(points)
        self.scatter.set_array(time_norm)
        self.start_marker.set_data([points[0, 0]], [points[0, 1]])
        self.current_marker.set_data([points[-1, 0]], [points[-1, 1]])
        self.ax.set_title('XY Trajectory Monitor (Points: %d)' % len(points))

        # 参数范围作为初始显示范围，超出时包含全部航迹并留出边距。
        for axis, initial_range, set_limits in (
                (0, self.x_range, self.ax.set_xlim),
                (1, self.y_range, self.ax.set_ylim)):
            lower = min(initial_range[0], points[:, axis].min())
            upper = max(initial_range[1], points[:, axis].max())
            padding = max((upper - lower) * 0.05, 0.1)
            set_limits(lower - padding, upper + padding)

        return self.scatter, self.start_marker, self.current_marker

    def run(self):
        """运行节点"""
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False,
            cache_frame_data=False)
        plt.show()
        
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = XYTrajectoryMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
