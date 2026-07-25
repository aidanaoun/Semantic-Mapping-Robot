#!/home/recon_car_ws/.venv/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.IMAGE_WIDTH = None
        self.IMAGE_HEIGHT = None
        self.curr_image_cv = None
        self.detection_results = None
        self.image_bridge = CvBridge()
        self.vision_model = YOLO("yolo26n.pt")

        
        self.camera_subscription = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)


    def camera_callback(self, msg):
        self.curr_image_cv = self.image_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.detection_results = self.vision_model.predict(self.curr_image_cv, conf=0.5)
        semantic_image = self.detection_results[0].plot()
        cv.imshow("Decection Image", semantic_image)
        cv.waitKey(10)

        return


def main(args=None):
    rclpy.init()
    obj_detection_node = ObjectDetectionNode()
    rclpy.spin(obj_detection_node)
    rclpy.shutdown()
    return