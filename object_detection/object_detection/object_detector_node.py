import json
import cv2 as cv
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO
from visualization_msgs.msg import Marker, MarkerArray


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("object_detection_node")
        self.declare_parameter("model_path", "yolo26n.pt")
        self.declare_parameter("camera_optical_frame", "camera_optical")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("camera_horizontal_fov", 1.396)
        self.declare_parameter("confidence_threshold", 0.60)
        self.declare_parameter("cluster_distance_threshold", 0.90)
        self.declare_parameter("cross_class_cluster_distance_threshold", 0.40)
        self.declare_parameter("minimum_sightings_to_publish", 5)
        self.declare_parameter("minimum_average_confidence_to_publish", 0.60)
        self.declare_parameter("candidate_timeout_sec", 2.0)
        self.declare_parameter("candidate_confirmation_window_sec", 4.0)
        self.declare_parameter("ignored_classes", ["traffic light", "stop sign", "parking meter", "lightpost", "light post", "traffic sign"])
        self.declare_parameter("marker_minimum_separation", 0.60)
        self.declare_parameter("marker_diameter", 0.26)
        self.declare_parameter("label_text_height", 0.30)

        self.model_path = self.get_parameter("model_path").value
        self.camera_optical_frame = self.get_parameter("camera_optical_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.camera_horizontal_fov = float(self.get_parameter("camera_horizontal_fov").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.cluster_distance_threshold = float(self.get_parameter("cluster_distance_threshold").value)
        self.cross_class_cluster_distance_threshold = float(self.get_parameter("cross_class_cluster_distance_threshold").value)
        self.minimum_sightings_to_publish = int(self.get_parameter("minimum_sightings_to_publish").value)
        self.minimum_average_confidence_to_publish = float(self.get_parameter("minimum_average_confidence_to_publish").value)
        self.candidate_timeout_ns = int(float(self.get_parameter("candidate_timeout_sec").value) * 1_000_000_000)
        self.candidate_confirmation_window_ns = int(float(self.get_parameter("candidate_confirmation_window_sec").value) * 1_000_000_000)
        self.ignored_classes = {str(name).strip().lower() for name in self.get_parameter("ignored_classes").value}
        self.marker_minimum_separation = float(self.get_parameter("marker_minimum_separation").value)
        self.marker_diameter = float(self.get_parameter("marker_diameter").value)
        self.label_text_height = float(self.get_parameter("label_text_height").value)

        self.image_bridge = CvBridge()
        self.vision_model = YOLO(self.model_path)

        self.camera_info = None
        self.object_clusters = []
        self.next_cluster_id = 0
        self.objects_added = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # CameraInfo is stored separately because the simulated camera intrinsics remain constant.
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/camera_info", self.camera_info_callback, qos_profile_sensor_data)
        self.image_sub = Subscriber(self, Image, "/camera/image_raw", qos_profile=qos_profile_sensor_data)
        self.scan_sub = Subscriber(self, LaserScan, "/scan", qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([self.image_sub, self.scan_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.synced_callback)

        semantic_map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.semantic_objects_pub = self.create_publisher(String, "/semantic_objects", semantic_map_qos)
        self.semantic_markers_pub = self.create_publisher(MarkerArray, "/semantic_markers", semantic_map_qos)
        self.clear_markers_on_next_publish = True
        self.visible_marker_ids = set()


    def camera_info_callback(self, camera_info_msg):
        self.camera_info = camera_info_msg


    def synced_callback(self, image_msg, lidar_msg):
        if self.camera_info is None:
            self.get_logger().warning("No /camera/camera_info received; using intrinsics calculated from camera_horizontal_fov", throttle_duration_sec=5.0)

        ranges = np.asarray(lidar_msg.ranges, dtype=np.float32)
        angles = lidar_msg.angle_min + np.arange(ranges.size, dtype=np.float32) * lidar_msg.angle_increment

        valid_ranges = np.isfinite(ranges) & (ranges >= lidar_msg.range_min) & (ranges <= lidar_msg.range_max)
        ranges, angles = ranges[valid_ranges], angles[valid_ranges]

        lidar_points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles), np.zeros_like(ranges))).astype(np.float32)

        if lidar_points.shape[0] == 0:
            return

        image_cv = self.image_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        image_height, image_width = image_cv.shape[:2]
        detection_results = self.vision_model.predict(image_cv, conf=self.confidence_threshold, verbose=False)
        current_result = detection_results[0]
        box_coords, box_names, box_confidences = self.result_tensors_to_arrays(current_result)

        lidar_time = Time.from_msg(lidar_msg.header.stamp)
        optical_points = self.lidar_to_cam_frame(lidar_points, lidar_msg.header.frame_id, lidar_time)
        if optical_points is None:
            return

        map_points = self.transform_points(lidar_points, lidar_msg.header.frame_id, self.map_frame, lidar_time)
        if map_points is None:
            return

        uv_points, valid_lidar_indices = self.optical_points_to_uv(optical_points, self.camera_info, image_width, image_height)

        display_image = current_result.plot()
        for u, v in np.rint(uv_points).astype(np.int32):
            cv.circle(display_image, (int(u), int(v)), 2, (0, 255, 0), -1)

        cv.imshow("YOLO detections and projected lidar points", display_image)
        cv.waitKey(1)

        if box_coords is None:
            self.publish_semantic_objects(lidar_time)
            return

        objects = self.localize_objects(box_coords, box_names, box_confidences, lidar_points, optical_points, map_points, uv_points, valid_lidar_indices)
        self.add_object_to_map(objects, lidar_time)



    def result_tensors_to_arrays(self, detection_result):
        boxes = detection_result.boxes
        if boxes is None or len(boxes) == 0:
            self.get_logger().info("No objects detected")
            return None, None, None

        box_coords = boxes.xyxy.detach().cpu().numpy().astype(np.float32)
        class_ids = boxes.cls.detach().cpu().numpy().astype(np.int32)
        box_confidences = boxes.conf.detach().cpu().numpy().astype(np.float32)
        box_names = np.asarray([detection_result.names[int(class_id)] for class_id in class_ids])
        return box_coords, box_names, box_confidences


    def lidar_to_cam_frame(self, raw_lidar_points, lidar_frame, time_stamp):
        return self.transform_points(raw_lidar_points, lidar_frame, self.camera_optical_frame, time_stamp)


    def transform_points(self, raw_points, source_frame, target_frame, time_stamp):
        if raw_points.ndim != 2 or raw_points.shape[1] != 3:
            raise ValueError("raw_points must have shape (N, 3)")

        try:
            frame_transform = self.tf_buffer.lookup_transform(target_frame, source_frame, time_stamp, timeout=Duration(seconds=0.1))
        except TransformException as error:
            self.get_logger().warning(f"Could not transform {source_frame} to {target_frame}: {error}", throttle_duration_sec=2.0)
            return None

        translation, rotation = frame_transform.transform.translation, frame_transform.transform.rotation

        quaternion = np.array([rotation.x, rotation.y, rotation.z, rotation.w], dtype=np.float32)
        quaternion_norm = np.linalg.norm(quaternion)

        if quaternion_norm < 1e-8:
            self.get_logger().error("Received an invalid zero-length TF quaternion")
            return None

        x, y, z, w = quaternion / quaternion_norm
        rotation_matrix = np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ], dtype=np.float32)

        translation_vector = np.array([translation.x, translation.y, translation.z], dtype=np.float32)
        return raw_points @ rotation_matrix.T + translation_vector



    def optical_points_to_uv(self, optical_points, camera_info_msg, image_width, image_height):
        if optical_points.ndim != 2 or optical_points.shape[1] != 3:
            raise ValueError("optical_points must have shape (N, 3)")
        if optical_points.shape[0] == 0:
            return np.empty((0, 2), dtype=np.float32), np.empty(0, dtype=np.int32)

        if camera_info_msg is not None:
            camera_matrix = np.asarray(camera_info_msg.k, dtype=np.float32).reshape(3, 3)
            fx, fy, cx, cy = camera_matrix[0, 0], camera_matrix[1, 1], camera_matrix[0, 2], camera_matrix[1, 2]
        else:
            fx = image_width / (2.0 * np.tan(self.camera_horizontal_fov / 2.0))
            fy = fx
            cx, cy = image_width / 2.0, image_height / 2.0

        if fx <= 0.0 or fy <= 0.0 or not np.isfinite([fx, fy, cx, cy]).all():
            self.get_logger().error("Camera intrinsics are invalid")
            return np.empty((0, 2), dtype=np.float32), np.empty(0, dtype=np.int32)

        # Optical-frame convention: X right, Y down, Z forward.
        X, Y, Z = optical_points[:, 0], optical_points[:, 1], optical_points[:, 2]

        valid_3d = (
            np.isfinite(X)
            & np.isfinite(Y)
            & np.isfinite(Z)
            & (Z > 0.0)
        )

        valid_lidar_indices = np.flatnonzero(valid_3d)
        X, Y, Z = X[valid_3d], Y[valid_3d], Z[valid_3d]

        u, v = fx * (X / Z) + cx, fy * (Y / Z) + cy

        inside_image = (
            (u >= 0.0)
            & (u < image_width)
            & (v >= 0.0)
            & (v < image_height)
        )

        uv_points = np.column_stack((u[inside_image], v[inside_image])).astype(np.float32)
        valid_lidar_indices = valid_lidar_indices[inside_image].astype(np.int32)

        return uv_points, valid_lidar_indices



    def localize_objects(self, box_coords, box_names, box_confidences, lidar_points, optical_points, map_points, uv_points, valid_lidar_indices):
        """Associate each YOLO box with the projected LiDAR point nearest its center."""
        objects = []

        for box, name, confidence in zip(box_coords, box_names, box_confidences):
            if str(name).strip().lower() in self.ignored_classes:
                continue

            x1, y1, x2, y2 = box

            points_inside_box = (
                (uv_points[:, 0] >= x1)
                & (uv_points[:, 0] <= x2)
                & (uv_points[:, 1] >= y1)
                & (uv_points[:, 1] <= y2)
            )

            projected_indices = np.flatnonzero(points_inside_box)
            if projected_indices.size == 0:
                continue

            box_center = np.array([(x1 + x2) / 2.0, (y1 + y2) / 2.0], dtype=np.float32)
            candidate_pixels = uv_points[projected_indices]
            squared_pixel_distances = np.sum((candidate_pixels - box_center) ** 2, axis=1)
            selected_projected_index = projected_indices[np.argmin(squared_pixel_distances)]
            selected_lidar_index = int(valid_lidar_indices[selected_projected_index])

            objects.append({
                "name": str(name),
                "confidence": float(confidence),
                "bbox": box.copy(),
                "pixel": uv_points[selected_projected_index].copy(),
                "lidar_point": lidar_points[selected_lidar_index].copy(),
                "optical_point": optical_points[selected_lidar_index].copy(),
                "map_point": map_points[selected_lidar_index].copy(),
            })

        return objects



    def add_object_to_map(self, objects, observation_time):
        """Merge new observations into persistent map-frame object clusters."""
        self.objects_added += len(objects)

        if objects:
            self.combine_sightings(objects, observation_time)

        self.publish_semantic_objects(observation_time)



    def combine_sightings(self, objects, observation_time):
        updated_cluster_ids = set()

        for detected_object in objects:
            map_point = detected_object["map_point"].astype(np.float64)
            matching_clusters = [cluster for cluster in self.object_clusters if cluster["id"] not in updated_cluster_ids]

            nearest_cluster = None
            nearest_distance = np.inf

            for cluster in matching_clusters:
                distance = float(np.linalg.norm(map_point[:2] - cluster["position"][:2]))
                matching_class = cluster["class_name"] == detected_object["name"]
                allowed_distance = self.cluster_distance_threshold if matching_class else self.cross_class_cluster_distance_threshold

                if distance <= allowed_distance and distance < nearest_distance:
                    nearest_cluster = cluster
                    nearest_distance = distance

            if nearest_cluster is None:
                observation_weight = max(detected_object["confidence"], 1e-6)
                new_cluster = {
                    "id": self.next_cluster_id,
                    "class_name": detected_object["name"],
                    "class_sighting_counts": {detected_object["name"]: 1},
                    "class_confidence_totals": {detected_object["name"]: detected_object["confidence"]},
                    "position": map_point.copy(),
                    "position_m2": np.zeros(3, dtype=np.float64),
                    "total_weight": observation_weight,
                    "average_confidence": detected_object["confidence"],
                    "sighting_count": 1,
                    "first_seen_ns": observation_time.nanoseconds,
                    "last_seen_ns": observation_time.nanoseconds,
                }

                self.object_clusters.append(new_cluster)
                updated_cluster_ids.add(new_cluster["id"])
                self.next_cluster_id += 1
                continue

            old_count = nearest_cluster["sighting_count"]
            new_count = old_count + 1
            observation_weight = max(detected_object["confidence"], 1e-6)
            new_total_weight = nearest_cluster["total_weight"] + observation_weight
            position_delta = map_point - nearest_cluster["position"]
            nearest_cluster["position"] += position_delta * observation_weight / new_total_weight
            nearest_cluster["position_m2"] += observation_weight * position_delta * (map_point - nearest_cluster["position"])
            nearest_cluster["total_weight"] = new_total_weight
            nearest_cluster["average_confidence"] = (nearest_cluster["average_confidence"] * old_count + detected_object["confidence"]) / new_count
            nearest_cluster["sighting_count"] = new_count
            nearest_cluster["last_seen_ns"] = observation_time.nanoseconds
            detected_class = detected_object["name"]
            nearest_cluster["class_sighting_counts"][detected_class] = nearest_cluster["class_sighting_counts"].get(detected_class, 0) + 1
            nearest_cluster["class_confidence_totals"][detected_class] = nearest_cluster["class_confidence_totals"].get(detected_class, 0.0) + detected_object["confidence"]
            nearest_cluster["class_name"] = max(nearest_cluster["class_sighting_counts"], key=lambda class_name: (nearest_cluster["class_sighting_counts"][class_name], nearest_cluster["class_confidence_totals"][class_name]))
            updated_cluster_ids.add(nearest_cluster["id"])



    def publish_semantic_objects(self, time_stamp):
        self.remove_stale_candidate_clusters(time_stamp)
        published_objects = []

        for cluster in self.object_clusters:
            if cluster["sighting_count"] < self.minimum_sightings_to_publish:
                continue
            if cluster["average_confidence"] < self.minimum_average_confidence_to_publish:
                continue

            if cluster["sighting_count"] > 1:
                position_std = np.sqrt(np.maximum(cluster["position_m2"] / cluster["total_weight"], 0.0))
            else:
                position_std = np.zeros(3, dtype=np.float64)

            published_objects.append({
                "id": cluster["id"],
                "class_name": cluster["class_name"],
                "position": {"x": float(cluster["position"][0]), "y": float(cluster["position"][1]), "z": float(cluster["position"][2])},
                "position_std": {"x": float(position_std[0]), "y": float(position_std[1]), "z": float(position_std[2])},
                "average_confidence": float(cluster["average_confidence"]),
                "total_weight": float(cluster["total_weight"]),
                "sighting_count": cluster["sighting_count"],
                "first_seen": self.nanoseconds_to_stamp_dict(cluster["first_seen_ns"]),
                "last_seen": self.nanoseconds_to_stamp_dict(cluster["last_seen_ns"]),
            })

        semantic_map = {
            "header": {"frame_id": self.map_frame, "stamp": self.nanoseconds_to_stamp_dict(time_stamp.nanoseconds)},
            "objects": published_objects,
        }

        message = String()
        message.data = json.dumps(semantic_map, separators=(",", ":"))
        self.semantic_objects_pub.publish(message)
        self.publish_semantic_markers(published_objects, time_stamp)


    def remove_stale_candidate_clusters(self, time_stamp):
        current_time_ns = time_stamp.nanoseconds

        self.object_clusters = [
            cluster for cluster in self.object_clusters
            if cluster["sighting_count"] >= self.minimum_sightings_to_publish
            or (
                current_time_ns - cluster["last_seen_ns"] <= self.candidate_timeout_ns
                and current_time_ns - cluster["first_seen_ns"] <= self.candidate_confirmation_window_ns
            )
        ]


    def select_non_overlapping_markers(self, published_objects):
        """Keep the strongest RViz marker when confirmed objects visually overlap."""
        if self.marker_minimum_separation <= 0.0:
            return published_objects

        prioritized_objects = sorted(published_objects, key=lambda semantic_object: (semantic_object["sighting_count"], semantic_object["average_confidence"]), reverse=True)
        selected_objects = []

        for semantic_object in prioritized_objects:
            position = semantic_object["position"]
            point = np.array([position["x"], position["y"]], dtype=np.float64)

            if all(np.linalg.norm(point - np.array([selected_object["position"]["x"], selected_object["position"]["y"]], dtype=np.float64)) >= self.marker_minimum_separation for selected_object in selected_objects):
                selected_objects.append(semantic_object)

        return sorted(selected_objects, key=lambda semantic_object: semantic_object["id"])


    def publish_semantic_markers(self, published_objects, time_stamp):
        marker_array = MarkerArray()
        stamp = time_stamp.to_msg()
        marker_objects = self.select_non_overlapping_markers(published_objects)
        current_marker_ids = {int(semantic_object["id"]) for semantic_object in marker_objects}

        if self.clear_markers_on_next_publish:
            clear_marker = Marker()
            clear_marker.action = Marker.DELETEALL
            marker_array.markers.append(clear_marker)
            self.clear_markers_on_next_publish = False

        for removed_id in self.visible_marker_ids - current_marker_ids:
            for namespace in ("semantic_objects", "semantic_object_labels"):
                delete_marker = Marker()
                delete_marker.header.frame_id = self.map_frame
                delete_marker.header.stamp = stamp
                delete_marker.ns = namespace
                delete_marker.id = removed_id
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)

        for semantic_object in marker_objects:
            object_id = int(semantic_object["id"])
            class_name = semantic_object["class_name"]
            position = semantic_object["position"]
            x, y, z = position["x"], position["y"], position["z"]

            object_marker = Marker()
            object_marker.header.frame_id = self.map_frame
            object_marker.header.stamp = stamp
            object_marker.ns = "semantic_objects"
            object_marker.id = object_id
            object_marker.type = Marker.CYLINDER
            object_marker.action = Marker.ADD
            object_marker.pose.position.x = x
            object_marker.pose.position.y = y
            object_marker.pose.position.z = z + 0.08
            object_marker.pose.orientation.w = 1.0
            object_marker.scale.x = self.marker_diameter
            object_marker.scale.y = self.marker_diameter
            object_marker.scale.z = 0.16
            object_marker.color.r = 0.10
            object_marker.color.g = 0.70
            object_marker.color.b = 1.00
            object_marker.color.a = 0.90
            marker_array.markers.append(object_marker)

            text_marker = Marker()
            text_marker.header.frame_id = self.map_frame
            text_marker.header.stamp = stamp
            text_marker.ns = "semantic_object_labels"
            text_marker.id = object_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = z + 0.26
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = self.label_text_height
            text_marker.color.r = 0.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0
            text_marker.text = class_name
            marker_array.markers.append(text_marker)

        self.semantic_markers_pub.publish(marker_array)
        self.visible_marker_ids = current_marker_ids



    @staticmethod
    def nanoseconds_to_stamp_dict(nanoseconds):
        return {"sec": int(nanoseconds // 1_000_000_000), "nanosec": int(nanoseconds % 1_000_000_000)}



def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()