import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Int32, Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import PointStamped
import cv2
import cv2.aruco as aruco
import numpy as np
import struct


# ArUco dictionary to use, e.g. DICT_4X4_50, DICT_4X4_250 (default matches the corridor tags).
DICTIONARY = os.getenv('ARUCO_DICTIONARY', 'DICT_4X4_250')
# Comma-separated list of marker IDs to report; empty (default) = report all detected.
ONLY_IDS = os.getenv('ARUCO_ONLY_IDS', '')

DICTS = {
    name: getattr(aruco, name) for name in dir(aruco) if name.startswith('DICT_')
}


def parse_only_ids(spec):
    if not spec.strip():
        return None
    return {int(part) for part in spec.split(',') if part.strip()}


class ArucoReader(Node):

    def __init__(self):
        super().__init__('aruco_reader')

        dictionary = DICTS.get(DICTIONARY)
        if dictionary is None:
            raise ValueError(
                'Unknown ARUCO_DICTIONARY "%s"; available: %s'
                % (DICTIONARY, ', '.join(sorted(DICTS))))
        self.dictionary = aruco.getPredefinedDictionary(dictionary)
        self.parameters = aruco.DetectorParameters_create()
        self.only_ids = parse_only_ids(ONLY_IDS)

        self.bridge = CvBridge()

        # Best-effort sensor QoS: images are a fire-hose, never want back-pressure
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Best (largest-area) marker — convenient for single-target consumers
        self.pub_id = self.create_publisher(Int32, 'aruco_detections', 10)
        self.pub_center = self.create_publisher(PointStamped, 'aruco_marker_center', 10)

        # All markers visible this frame
        self.pub_ids = self.create_publisher(Int32MultiArray, 'aruco_marker_ids', 10)
        self.pub_areas = self.create_publisher(Float32MultiArray, 'aruco_marker_areas', 10)
        self.pub_centers = self.create_publisher(PointCloud2, 'aruco_marker_centers', 10)
        self.pub_corners = self.create_publisher(Float32MultiArray, 'aruco_marker_corners', 10)

        self.sub = self.create_subscription(
            Image, 'image', self.image_cb, sensor_qos)

        self._log_counter = 0
        self._last_best = -1
        self.get_logger().info(
            'ArUco reader started (%s, only_ids=%s)'
            % (DICTIONARY, sorted(self.only_ids) if self.only_ids else 'all'))

    def _make_pointcloud2(self, centers, stamp, frame_id):
        """Build a PointCloud2 where each point is (cx, cy, area) in pixel space."""
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(centers)
        msg.is_bigendian = False
        msg.is_dense = True
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.fields = fields
        msg.point_step = 12
        msg.row_step = 12 * len(centers)
        data = b''
        for cx, cy, area in centers:
            data += struct.pack('<fff', float(cx), float(cy), float(area))
        msg.data = data
        return msg

    def image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn('CV bridge error: %s' % e)
            return
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        if ids is not None and len(ids) > 0:
            areas = [cv2.contourArea(c[0].astype(np.float32)) for c in corners]
            all_ids = [int(i[0]) for i in ids]
            all_centers = []
            for c in corners:
                pts = c[0]
                all_centers.append((float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1]))))

            if self.only_ids is not None:
                keep = [k for k, i in enumerate(all_ids) if i in self.only_ids]
                areas = [areas[k] for k in keep]
                all_ids = [all_ids[k] for k in keep]
                all_centers = [all_centers[k] for k in keep]
                corners = [corners[k] for k in keep]

        if ids is None or len(ids) == 0 or len(all_ids) == 0:
            # No marker visible — publish -1 sentinel so downstream nodes
            # don't retain stale values
            self.pub_id.publish(Int32(data=-1))
            self.pub_ids.publish(Int32MultiArray(data=[]))
            self.pub_areas.publish(Float32MultiArray(data=[]))
            self.pub_centers.publish(
                self._make_pointcloud2([], msg.header.stamp, msg.header.frame_id))
            self.pub_corners.publish(Float32MultiArray(data=[]))

            self._log_counter += 1
            if self._log_counter >= 60:
                self.get_logger().info('No markers visible')
                self._log_counter = 0
            self._last_best = -1
            return

        best_idx = int(np.argmax(areas))
        best_id = all_ids[best_idx]
        best_cx, best_cy = all_centers[best_idx]
        best_area = float(areas[best_idx])

        self.pub_id.publish(Int32(data=best_id))
        center_msg = PointStamped()
        center_msg.header.stamp = msg.header.stamp
        center_msg.header.frame_id = msg.header.frame_id
        center_msg.point.x = best_cx
        center_msg.point.y = best_cy
        center_msg.point.z = 0.0
        self.pub_center.publish(center_msg)

        self.pub_ids.publish(Int32MultiArray(data=all_ids))
        self.pub_areas.publish(Float32MultiArray(data=[float(a) for a in areas]))
        centers_with_area = [(cx, cy, float(a)) for (cx, cy), a in zip(all_centers, areas)]
        self.pub_centers.publish(
            self._make_pointcloud2(centers_with_area, msg.header.stamp, msg.header.frame_id))

        best_corners = corners[best_idx][0]
        corner_data = []
        for pt in best_corners:
            corner_data.append(float(pt[0]))
            corner_data.append(float(pt[1]))
        self.pub_corners.publish(Float32MultiArray(data=corner_data))

        self._log_counter += 1
        if best_id != self._last_best or self._log_counter >= 30:
            self.get_logger().info(
                'Saw %d marker(s): %s, best=%d (area=%.0f, center=(%.0f,%.0f))'
                % (len(all_ids), all_ids, best_id, best_area, best_cx, best_cy))
            self._log_counter = 0
        self._last_best = best_id


def main(args=None):
    rclpy.init(args=args)
    node = ArucoReader()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
