#!/usr/bin/env python3
"""记录 /ring_center 的 x 值，Ctrl+C 后作图：横轴时间(s)，纵轴 x(m)。"""

import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped

times = []
x_values = []
start_time = None

def callback(msg):
    global start_time
    t = (msg.header.stamp - start_time).to_sec()
    times.append(t)
    x_values.append(msg.pose.position.x)
    rospy.loginfo(f"[{t:6.2f}s] x = {msg.pose.position.x:.3f}")

def main():
    global start_time
    rospy.init_node("plot_ring_center_x")
    start_time = rospy.Time.now()
    rospy.Subscriber("/ring_center", PoseStamped, callback)
    rospy.loginfo("Recording /ring_center x values. Press Ctrl+C to stop and plot.")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        if times:
            plt.figure(figsize=(10, 4))
            plt.plot(times, x_values, marker=".", linestyle="-", markersize=3)
            plt.xlabel("Time (s)")
            plt.ylabel("X (m)")
            plt.title("Ring Center X over Time")
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
            save_path = rospy.get_param("~save_path", "/home/a/craic_catkin_ws/src/maxt_pkg/log/ring_center_x.png")
            plt.savefig(save_path, dpi=150)
            rospy.loginfo(f"Saved plot to {save_path} ({len(times)} samples)")
            plt.show()
