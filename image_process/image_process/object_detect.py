#!/usr/bin/python3

from logging import info
import rclpy
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from example_interfaces.msg import Bool
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image
import torch
from cv_bridge import CvBridge

class ObjectDetector(Node):
    def __init__(self):
        super().__init__("object_detecctor")
        # self.robot_name = "object_detecctor"
        # self.number = 1
        self.img_feed = self.create_subscription(Image,"camera/image_raw",self.imgfeed_callback,2)
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/ros2/ros2_ws/src/image_process/resource/best.pt') 
        print("Loaded model")
        self.model.conf = 0.25  # NMS confidence threshold
        self.h_total_area = 360*240
        self.h_h = 240
        self.h_w = 320
        self.yol_pub = self.create_publisher(Bool,'/yolo/procc',2)
        # iou = 0.45  # NMS IoU threshold
        # agnostic = False  # NMS class-agnostic
        # multi_label = False  # NMS multiple labels per box
        # classes = None  # (optional list) filter by class, i.e. = [0, 15, 16] for COCO persons, cats and dogs
        self.model.max_det = 2 # maximum number of detections per image
        # amp = False  # Automatic Mixed Precision (AMP) inference
        self.br = CvBridge()
    
    def imgfeed_callback(self,data):
        self.get_logger().info("Got Image")
		# process image
        current_frame = self.br.imgmsg_to_cv2(data)
        processed_image = self.model(current_frame)
        processed_image.show()
        pi_frames = processed_image.pandas().xyxy[0]
        if not (pi_frames.size == 0):
            width = max(pi_frames.iloc[0][2], 480)-max(pi_frames.iloc[0][0],120)
            height = max(340,pi_frames.iloc[0][3])- max(100,pi_frames.iloc[0][1])
            print(width, "  ", height)
            prob = width*height/ self.h_total_area

        if prob > 0.35:
            self.get_logger().info("Detected Object")
            d = Bool()
            d.data = True
            self.yol_pub.publish(Bool,d)


    


def main():
    rclpy.init(args = None)
    node  = ObjectDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
