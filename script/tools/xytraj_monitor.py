#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
        
        # 存储轨迹数据
        self.x_data = []
        self.y_data = []
        self.timestamps = []
        
        # 位置消息订阅
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)
        
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
        
        rospy.loginfo("XY Trajectory Monitor started!")
        rospy.loginfo("Subscribing to: %s", self.pose_topic)
        
    def pose_callback(self, msg):
        """处理位置消息"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        t = msg.header.stamp.to_sec()
        
        self.x_data.append(x)
        self.y_data.append(y)
        self.timestamps.append(t)
        
    def update_plot(self, frame):
        """更新图形"""
        if len(self.x_data) < 2:
            return self.scatter,
        
        self.ax.clear()
        self.ax.set_xlim(self.x_range)
        self.ax.set_ylim(self.y_range)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('XY Trajectory Monitor (Points: %d)' % len(self.x_data))
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        
        # 计算时间归一化值 (0-1)
        t_min = min(self.timestamps)
        t_max = max(self.timestamps)
        if t_max > t_min:
            time_norm = [(t - t_min) / (t_max - t_min) for t in self.timestamps]
        else:
            time_norm = [0.0] * len(self.timestamps)
        
        # 绘制轨迹
        self.scatter = self.ax.scatter(self.x_data, self.y_data, 
                                      c=time_norm, 
                                      cmap='hot',
                                      s=self.point_size,
                                      alpha=0.8,
                                      vmin=0, vmax=1)
        
        # 标记起点和终点
        self.ax.plot(self.x_data[0], self.y_data[0], 'go', markersize=10, label='Start')
        self.ax.plot(self.x_data[-1], self.y_data[-1], 'r*', markersize=12, label='Current')
        self.ax.legend(loc='upper right')
        
        return self.scatter,
        
    def run(self):
        """运行节点"""
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)
        plt.show()
        
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = XYTrajectoryMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
