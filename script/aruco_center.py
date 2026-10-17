#!/usr/bin/env python3
"""订阅 ROS 1 图像，识别指定 ArUco，并发布其中心像素坐标。

数据流程：Image 消息 → OpenCV 图像 → 灰度图 → 四角点 → 中心 → PointStamped。
运行示例：
    roslaunch maxt_pkg aruco_center.launch
    # 上述方式自动加载 config/aruco_center.yaml；修改配置后重启节点。
    # 也可以直接运行，使用脚本默认值或命令行指定私有参数：
    rosrun maxt_pkg aruco_center.py _image_topic:=/camera/image_raw _marker_id:=0

输出话题（节点名为 aruco_center 时）：
    /aruco_center/center：point.x = u，point.y = v，point.z = 0。
    /aruco_center/is_found：std_msgs/Bool，每帧发布指定目标是否被找到。
    /aruco_center/debug_image：画出标记边框和中心的图像。
像素原点在左上角，u 向右增大，v 向下增大，坐标可以是小数。
本脚本只做二维定位，不计算距离或三维姿态，因此不需要标记实际尺寸。
"""

import cv2  # OpenCV：图像处理、ArUco 检测及绘图。
import numpy as np  # 数组运算：计算两条对角线的交点。
import rospy  # ROS 1 的 Python 接口：节点、参数、话题和日志。
import actionlib
import threading
from maxt_pkg.msg import GuidanceAction, GuidanceFeedback, GuidanceResult
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class ArucoCenter:
    """将检测器、ROS 通信对象和每帧图像的处理过程封装到一个类中。"""

    def __init__(self):
        # CvBridge 负责 ROS Image 消息与 OpenCV 的 NumPy 图像数组互转。
        self.bridge = CvBridge()
        # “~”表示节点私有参数，例如 /aruco_center/marker_id。
        # launch 将 YAML 加载到私有命名空间，这里通过 get_param 读取。
        # 第二个参数是配置缺失时的默认值；直接 rosrun 时可用 _marker_id:=23。
        self.marker_id = int(rospy.get_param("~marker_id", 0))
        # 字典必须与实际标记一致。4X4 表示内部编码为 4×4 个单元，
        # 50 表示字典有 50 个标记（ID 0～49），并不是图片的像素大小。
        dictionary_name = rospy.get_param("~dictionary", "DICT_4X4_50")
        # 部分 OpenCV 安装不包含 ArUco，提前检查可以给出清晰的错误信息。
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("OpenCV lacks aruco; install python3-opencv with ArUco support")
        # 将字符串名称转换成 OpenCV 常量；属性不存在时返回 None。
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if dictionary_id is None or not dictionary_name.startswith("DICT_"):
            raise ValueError("Unknown ArUco dictionary: " + dictionary_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        # 兼容两套 OpenCV API：新版使用 ArucoDetector 对象，
        # 旧版（例如本机 OpenCV 4.2）使用 detectMarkers 函数。
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary)
        else:
            self.detector = None
            self.parameters = cv2.aruco.DetectorParameters_create()
        # 私有话题随节点名称变化。queue_size=1 限制待发送消息数量，
        # 避免积压旧结果；这并不保证整个系统没有延迟。
        # PointStamped 在坐标之外还有 header，可保留原图时间戳和 frame_id。
        # 输出话题也从配置读取，可使用绝对路径或节点私有名称。
        center_topic = rospy.get_param("~center_topic", "~center")
        debug_image_topic = rospy.get_param("~debug_image_topic", "~debug_image")
        is_found_topic = rospy.get_param("~is_found_topic", "~is_found")
        self.center_pub = rospy.Publisher(center_topic, PointStamped, queue_size=1)
        self.image_pub = rospy.Publisher(debug_image_topic, Image, queue_size=1)
        self.is_found_pub = rospy.Publisher(is_found_topic, Bool, queue_size=1)
        self.feedback_lock = threading.Lock()
        self.feedback_stamp = None
        self.feedback = GuidanceFeedback(target_visible=False)
        self.feedback_timeout = float(rospy.get_param("~feedback_timeout", 0.3))
        # 可选真实视觉 Action 服务；独立识别入口发布图像、中心及检测状态。
        if rospy.get_param("~run_guidance", False):
            self.server = actionlib.SimpleActionServer(
                "/guidance", GuidanceAction, execute_cb=self.execute_guidance,
                auto_start=False)
            self.server.start()
        topic = rospy.get_param("~image_topic", "/camera/image_raw")
        # 收到图像时调用 self.callback(msg)，不需要手动写读取图像的循环。
        # buff_size 是接收缓冲区字节数，2**24 为 16 MiB，适合较大的图像消息。
        self.subscriber = rospy.Subscriber(
            topic, Image, self.callback, queue_size=1, buff_size=2**24
        )
        rospy.loginfo("Detecting ArUco ID %d (%s) on %s", self.marker_id, dictionary_name, topic)

    def callback(self, msg):
        """处理一帧图像；找到目标时发布一次中心，未找到时不发布坐标。"""
        frame_feedback = GuidanceFeedback(target_visible=False)
        try:
            # 将输入编码统一成 BGR 三通道，每个通道 8 位。
            # 即使输入是 mono8 灰度图，CvBridge 也可以转换成 bgr8。
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            # 无法处理本帧时报告 false，避免下游继续沿用上一帧的 true。
            self.is_found_pub.publish(Bool(data=False))
            with self.feedback_lock:
                self.feedback = frame_feedback
                self.feedback_stamp = msg.header.stamp
            # 转换失败时跳过本帧；throttle 使此日志最多每 2 秒输出一次。
            rospy.logwarn_throttle(2.0, "Image conversion failed: %s" % exc)
            return
        # 检测使用灰度图；保留彩色 image 用来画调试结果。
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # corners：每个标记对应四个有序角点，常见数组形状为 (1, 4, 2)。
        # ids：对应的 ID，常见形状为 (N, 1)；没有检测结果时为 None。
        # 第三个返回值是未通过编码验证的候选轮廓，当前用“_”忽略。
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.parameters
            )
        found = False
        if ids is not None:
            # flatten 将 ID 数组展平成一维；zip 将角点和对应 ID 配对。
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                # 图像中可能有多个标记，只处理指定 ID。
                if int(marker_id) != self.marker_id:
                    continue
                points = marker_corners.reshape(4, 2)
                # 将角点整理成四行 [u, v]。角点沿标记边界有序排列，
                # 因此 0 与 2、1 与 3 分别是两条对角线的端点。
                # 透视下标记可能成为不对称四边形，四角点平均值通常不等于
                # 实际中心的投影，所以这里取对角线交点。
                # 该几何关系基于平面透视模型；镜头畸变较大时仍会产生误差。
                # 齐次坐标把 [u, v] 写成 [u, v, 1]，便于统一表示点和直线。
                homogeneous = np.column_stack((points, np.ones(4)))
                # 两个齐次点叉乘得到直线 [a, b, c]：a*u + b*v + c = 0。
                diagonal_a = np.cross(homogeneous[0], homogeneous[2])
                diagonal_b = np.cross(homogeneous[1], homogeneous[3])
                # 两条直线叉乘得到交点 [U, V, W]。
                center = np.cross(diagonal_a, diagonal_b)
                # W 接近 0 表示没有稳定的有限交点，跳过以避免除零。
                if abs(center[2]) < 1e-9:
                    continue
                # 齐次坐标归一化：u = U/W，v = V/W，保留亚像素小数。
                u, v = center[:2] / center[2]
                output = PointStamped()
                # 使用图像采集时刻，而不是处理完成时刻，便于下游判断数据时效。
                # 注意：虽然沿用图像 frame_id，但 x/y 的单位是像素，
                # 不是该坐标系下以米为单位的三维点，不能直接用 TF 变换。
                output.header = msg.header
                output.point.x = float(u)
                output.point.y = float(v)
                output.point.z = 0.0
                self.center_pub.publish(output)
                frame_feedback = GuidanceFeedback(
                    target_visible=True,
                    error_u=float(u - (msg.width - 1) / 2.0),
                    error_v=float(v - (msg.height - 1) / 2.0))
                # 只限制终端日志频率，中心坐标仍在每个成功检测的回调中发布。
                rospy.loginfo_throttle(1.0, "ArUco %d center: (%.1f, %.1f) px" % (marker_id, u, v))
                found = True
                # 仅有调试图像订阅者时才复制图像、绘图，减少额外计算。
                if self.image_pub.get_num_connections():
                    image = image.copy()
                    cv2.aruco.drawDetectedMarkers(image, [marker_corners])
                    # 绘图位置必须是整数，所以 round；发布的坐标仍保留小数。
                    # BGR 中 (0, 0, 255) 是红色，半径 5，厚度 -1 表示实心圆。
                    cv2.circle(image, (round(u), round(v)), 5, (0, 0, 255), -1)
                # 若同一帧出现多个相同 ID，只使用第一个有效结果。
                break
        # 每个正常处理的图像回调恰好发布一次，不受调试图像订阅情况影响。
        # true 表示指定 ID 已被检测到且成功计算中心，不表示检测到任意标记。
        # Bool 没有时间戳；如果输入图像停止，状态也停止发布，下游仍需超时判断。
        self.is_found_pub.publish(Bool(data=found))
        with self.feedback_lock:
            self.feedback = frame_feedback
            self.feedback_stamp = msg.header.stamp
        if not found:
            # 不发布 (0, 0) 来表示丢失，因为它也是有效像素位置。
            # 下游控制节点应检查最后一次坐标的时间戳，超时后停止使用旧结果。
            rospy.logwarn_throttle(2.0, "ArUco ID %d not detected" % self.marker_id)
        if self.image_pub.get_num_connections():
            # 转回 ROS Image 并保留原图 header。未找到时也发送未标注的原图，
            # 方便判断目标是否在视野内。
            debug = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            debug.header = msg.header
            self.image_pub.publish(debug)

    def execute_guidance(self, goal):
        if goal.target_id != str(self.marker_id):
            self.server.set_aborted(GuidanceResult(message="Unknown ArUco ID"))
            return
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.server.set_preempted(GuidanceResult(message="PREEMPTED"))
                return
            with self.feedback_lock:
                age = ((rospy.Time.now() - self.feedback_stamp).to_sec()
                       if self.feedback_stamp is not None else float("inf"))
                feedback = GuidanceFeedback(
                    target_visible=self.feedback.target_visible and
                    0.0 <= age <= self.feedback_timeout,
                    error_u=self.feedback.error_u, error_v=self.feedback.error_v)
            self.server.publish_feedback(feedback)
            rate.sleep()


if __name__ == "__main__":
    # 直接运行脚本时进入这里；作为模块导入时不自动启动 ROS 节点。
    rospy.init_node("aruco_center")
    try:
        # 构造对象时初始化检测器并注册发布者、订阅者。
        node = ArucoCenter()
    except (RuntimeError, ValueError) as exc:
        # 初始化配置错误时输出致命日志并以非零状态退出。
        rospy.logfatal(str(exc))
        raise SystemExit(1)
    # 保持节点运行，由 rospy 接收消息并调用回调，直到节点关闭。
    rospy.spin()
