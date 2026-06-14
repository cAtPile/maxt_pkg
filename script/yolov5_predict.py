#!/home/a/yolov5_venv/bin/python3
# -*- coding: utf-8 -*-

import rospy
import os
import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path
from maxt_pkg.msg import StringStamped
import torch
import torch.nn.functional as F
from std_srvs.srv import Empty, EmptyResponse


ROOT = ''
if __name__ == '__main__':
    rospy.init_node('yolov5_predict_node', sys.argv)
    ROOT = rospy.get_param('~yolov5_root')
else:
    FILE = Path(__file__).resolve()
    ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))


from models.common import DetectMultiBackend
from utils.augmentations import classify_transforms
from utils.general import LOGGER, check_requirements, colorstr, increment_path, print_args, set_logging
from utils.torch_utils import select_device, smart_inference_mode, time_sync


@smart_inference_mode()
def detect(
        weights, #weightpath
        source,  # image
        imgsz=64,  # inference size
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
        confidence=0.85  # Confidence Threshold
):
    file = str(source)
    seen, dt = 1, [0.0, 0.0, 0.0]
    device = select_device(device)
    
    # Transforms
    transforms = classify_transforms(imgsz)

    # Load model
    model = DetectMultiBackend(weights, device=device, dnn=dnn, fp16=half)
    model.warmup(imgsz=(1, 3, imgsz, imgsz))  # warmup

    # Image
    t1 = time_sync()
    im = cv2.cvtColor(source, cv2.COLOR_BGR2RGB)
    im = transforms(im).unsqueeze(0).to(device)
    im = im.half() if model.fp16 else im.float()
    t2 = time_sync()
    dt[0] += t2 - t1

    # Inference
    results = model(im)
    t3 = time_sync()
    dt[1] += t3 - t2

    p = F.softmax(results, dim=1)  # probabilities
    top5_indices = p.argsort(1, descending=True)[:, :5].squeeze()  # top 5 indices
    top5_confidences = p[0, top5_indices]  # top 5 confidences

    #Log top 5 class names and their confidences
    top5_classes = [model.names[idx] for idx in top5_indices]
    message = "Top 5 classes:\n"
    for cls, conf in zip(top5_classes, top5_confidences):
        message = message + f"{cls:<10}:{conf:.3f}\n"
    rospy.loginfo(message)

    return model.names[top5_indices[0]] if torch.max(p) > confidence else ''


def cut_circumscribed_square(origin_image):
    #cv2.imshow('origin', origin_image)
    image = cv2.cvtColor(origin_image, cv2.COLOR_BGR2GRAY)
    image = cv2.GaussianBlur(image, (21, 21), 0)
    height, width = image.shape
    _, image = cv2.threshold(src=image, thresh=100, maxval=255,
                             type=cv2.THRESH_BINARY)
    #cv2.imshow('binary', image)
    result = cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, 1, int(max(width, height)), param1=1350,
                              param2=20, minRadius=50,
                              maxRadius=int(max(width, height)/2))
    if result is not None:
        result = np.uint16(np.around(result).reshape(-1, 3))
        x, y, r = result[0]
        r = r*0.66
        min_x = int(max(0, x-r))
        max_x = int(min(width-1, x+r))
        min_y = int(max(0, y-r))
        max_y = int(min(height-1, y+r))
        if min_x >= max_x or min_y >= max_y:
            return None
        cut = origin_image[min_y:max_y, min_x:max_x] 
        return cut
    return None


def image_cb(msg, bridge, weights, device, pub, confidence):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    input = cut_circumscribed_square(cv_image)
    if input is not None:
        #cv2.imshow("detect input", input)
        #cv2.waitKey(1)
        name = detect(weights=weights, source=input, device=device, confidence=confidence)
        if name != '':
            result = StringStamped()
            result.header = msg.header
            result.data.append(name)
            pub.publish(result)


subscriber = None

def toggleSubscribe(bridge, weights, device, str_pub, confidence):
    global subscriber
    if subscriber is None:
        rospy.loginfo("enable yolo_detect")
        subscriber = rospy.Subscriber('camera/image_raw', Image,
            lambda msg: image_cb(msg, bridge, weights, device, str_pub, confidence)
        )
    else:
        rospy.loginfo("disable yolo_detect")
        subscriber.unregister()
        subscriber = None
    return EmptyResponse()


def main():
    global subscriber
    set_logging(verbose=False)
    weights = rospy.get_param('~weights_path')
    device = rospy.get_param('~device', '')
    confidence = rospy.get_param('~confidence', 0.85)
    always_on = rospy.get_param("~always_on", False)
    bridge = CvBridge()
    str_pub = rospy.Publisher('yolo_detect', StringStamped, queue_size=1)
    if always_on:
        subscriber = rospy.Subscriber('camera/image_raw', Image,
            lambda msg: image_cb(msg, bridge, weights, device, str_pub, confidence)
        )
    else:
        rospy.Service('toggle_yolo_detect', Empty, lambda _:toggleSubscribe(bridge, weights, device, str_pub, confidence))
    rospy.spin()


if __name__ == '__main__':
    main()