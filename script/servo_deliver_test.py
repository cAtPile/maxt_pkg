#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty, Int32


def make_empty():
    return Empty()


rospy.init_node("servo_deliver_test")

# --- 全部仓门 ---
pub_all_open = rospy.Publisher("/servo/all/open", Empty, queue_size=1)
pub_all_close = rospy.Publisher("/servo/all/close", Empty, queue_size=1)

# --- 左前 ---
pub_fl_angle = rospy.Publisher("/servo/front_left", Int32, queue_size=1)
pub_fl_open = rospy.Publisher("/servo/front_left/open", Empty, queue_size=1)
pub_fl_close = rospy.Publisher("/servo/front_left/close", Empty, queue_size=1)

# --- 右前 ---
pub_fr_angle = rospy.Publisher("/servo/front_right", Int32, queue_size=1)
pub_fr_open = rospy.Publisher("/servo/front_right/open", Empty, queue_size=1)
pub_fr_close = rospy.Publisher("/servo/front_right/close", Empty, queue_size=1)

# --- 右后 ---
pub_br_angle = rospy.Publisher("/servo/back_right", Int32, queue_size=1)
pub_br_open = rospy.Publisher("/servo/back_right/open", Empty, queue_size=1)
pub_br_close = rospy.Publisher("/servo/back_right/close", Empty, queue_size=1)

rospy.sleep(1.0)  # 等待发布者注册完成

# ========== 测试序列 ==========

# 1. 打开全部仓门
rospy.loginfo("[1/7] Opening all doors...")
pub_all_open.publish(make_empty())
rospy.sleep(2.0)

# 2. 关闭全部
rospy.loginfo("[2/7] Closing all doors...")
pub_all_close.publish(make_empty())
rospy.sleep(2.0)

# 3. 单独打开左前
rospy.loginfo("[3/7] Opening front_left...")
pub_fl_open.publish(make_empty())
rospy.sleep(2.0)
pub_fl_close.publish(make_empty())

# 4. 单独打开右前
rospy.loginfo("[4/7] Opening front_right...")
pub_fr_open.publish(make_empty())
rospy.sleep(2.0)
pub_fr_close.publish(make_empty())

# 5. 单独打开右后
rospy.loginfo("[5/7] Opening back_right...")
pub_br_open.publish(make_empty())
rospy.sleep(2.0)
pub_br_close.publish(make_empty())

# 6. 全部打开
rospy.loginfo("[6/7] Opening all doors...")
pub_all_open.publish(make_empty())
rospy.sleep(2.0)

# 7. 全部关闭（复位）
rospy.loginfo("[7/7] Closing all doors (reset)...")
pub_all_close.publish(make_empty())

rospy.loginfo("Servo delivery test complete.")
