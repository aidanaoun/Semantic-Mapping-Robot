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
        curr_detection_result = self.detection_results[0]

        cv.imshow("image", curr_detection_result.plot())
        cv.waitKey(1)

        box_coords, box_names = self.result_tensors_to_arrays(curr_detection_result)
        if box_coords is None or box_names is None:
            return
        
        self.localize_objects()
        self.add_object_to_map()
        return


    def result_tensors_to_arrays(self, detection_result):
        boxes = detection_result.boxes
        if boxes is None or len(boxes) == 0:
            self.get_logger().info("No objects detected")
            return [None, None]
        
        boxes_tensor = boxes.xyxy
        class_ids_tensor = boxes.cls
        box_count = boxes_tensor.size(dim=0)
        box_coords = np.ndarray((box_count, 4))
        box_names = []
        
        for i in range(box_count):
            box_coords[i][0] = boxes_tensor[i, 0].item()
            box_coords[i][1] = boxes_tensor[i, 1].item()
            box_coords[i][2] = boxes_tensor[i, 2].item()
            box_coords[i][3] = boxes_tensor[i, 3].item()

            cur_class_id = int(class_ids_tensor[i].item())
            box_names.append(detection_result.names[cur_class_id])
            self.get_logger().info(f" Box coordinates {box_names[i]}: x1={box_coords[i][0]}, y1={box_coords[i][1]}, {box_coords[i][2]}, {box_coords[i][3]}")

        box_names = np.array(box_names)
        return [box_coords, box_names]


    def localize_objects(self):
        return


    def add_object_to_map(self):
        return


def main(args=None):
    rclpy.init()
    obj_detection_node = ObjectDetectionNode()
    rclpy.spin(obj_detection_node)
    rclpy.shutdown()
    return