#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import TwistStamped
import threading

class VelocityMonitor:
    def __init__(self):
        rospy.init_node('velocity_monitor', anonymous=True)
        
        self.lock = threading.Lock()
        
        # 获取参数
        self.velocity_topic = rospy.get_param('~velocity_topic', '/mavros/local_position/velocity_local')
        self.time_window = rospy.get_param('~time_window', 10.0) 
        
        # 存储数据
        self.timestamps = []
        self.vx_data = []
        self.vy_data = []
        self.vz_data = []
        self.speed_data = []
        
        # 初始化画布
        self.fig, self.axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
        self.lines = []
        colors = ['r', 'g', 'b', 'k']
        labels = ['VX (m/s)', 'VY (m/s)', 'VZ (m/s)', 'Speed (m/s)']
        
        for i in range(4):
            line, = self.axes[i].plot([], [], color=colors[i], linewidth=1.5)
            self.lines.append(line)
            self.axes[i].set_ylabel(labels[i])
            self.axes[i].grid(True, alpha=0.3)
            self.axes[i].set_ylim(-2, 2) 

        self.axes[3].set_xlabel('Time Offset (s)')
        self.fig.suptitle('UAV Velocity Real-time Monitor')
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        # 订阅速度话题
        self.vel_sub = rospy.Subscriber(self.velocity_topic, TwistStamped, self.velocity_callback)
        
        # 重要：将 animation 对象存为类成员，防止被 Python 垃圾回收导致动画停止
        self.ani = None 

    def velocity_callback(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z
        speed = np.sqrt(vx**2 + vy**2 + vz**2)
        now = rospy.get_time()

        with self.lock:
            self.timestamps.append(now)
            self.vx_data.append(vx)
            self.vy_data.append(vy)
            self.vz_data.append(vz)
            self.speed_data.append(speed)

            # 保持窗口大小
            while self.timestamps and (now - self.timestamps[0] > self.time_window):
                self.timestamps.pop(0)
                self.vx_data.pop(0)
                self.vy_data.pop(0)
                self.vz_data.pop(0)
                self.speed_data.pop(0)

    def update_plot(self, frame):
        # 检查 ROS 是否还在运行，不在运行就关闭图表
        if rospy.is_shutdown():
            plt.close('all')
            return self.lines

        with self.lock:
            if not self.timestamps:
                return self.lines

            now = rospy.get_time()
            t_rel = [t - now for t in self.timestamps]
            data_list = [self.vx_data, self.vy_data, self.vz_data, self.speed_data]
            
            for i in range(4):
                if len(t_rel) == len(data_list[i]):
                    self.lines[i].set_data(t_rel, data_list[i])
                    
                    # 动态缩放 Y 轴
                    if len(data_list[i]) > 0:
                        d_min, d_max = min(data_list[i]), max(data_list[i])
                        margin = max(0.2, (d_max - d_min) * 0.2)
                        self.axes[i].set_ylim(d_min - margin, d_max + margin)
            
            self.axes[-1].set_xlim(-self.time_window, 0)
            
        return self.lines

    def run(self):
        # 这里的关键是把返回的对象赋值给 self.ani
        # cache_frame_data=False 可以稍微减少内存占用
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=50, blit=False, cache_frame_data=False
        )
        
        # 只有在主线程调用 plt.show()
        # 它会自动处理 GUI 刷新，不需要显式调用 rospy.spin()
        # 因为 Subscriber 的回调是在后台线程运行的
        plt.show()

if __name__ == '__main__':
    try:
        monitor = VelocityMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass