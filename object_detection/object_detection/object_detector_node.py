import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
import matplotlib as plt
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.IMAGE_WIDTH = None
        self.IMAGE_HEIGHT = None
        self.curr_image_cv = None

        self.image_bridge = CvBridge()
        self.camera_subscription = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)


    def camera_callback(self, msg):
        self.curr_image_cv = self.image_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv.imshow('current image', self.curr_image_cv)
        cv.waitKey(10)
        return


def main(args=None):
    rclpy.init()
    obj_detection_node = ObjectDetectionNode()
    rclpy.spin(obj_detection_node)
    rclpy.shutdown()
    return