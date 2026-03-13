#!/usr/bin/env python3
"""
ROS 2 node that bridges Record3D iPhone streaming data into the topics
required by NVIDIA Isaac ROS FoundationPose:

Published topics
────────────────
  /iphone/color/image_raw          sensor_msgs/Image        (BGR8)
  /iphone/depth/image_raw          sensor_msgs/Image        (32FC1, metres)
  /iphone/camera_info              sensor_msgs/CameraInfo   (K matrix + distortion)
  /iphone/segmentation/mask        sensor_msgs/Image        (MONO8 – placeholder)

TF broadcast
────────────
  world → iphone_camera   (from Record3D camera_pose)

All messages carry the same stamp so that Isaac ROS exact‐time synchronisers
can match them.
"""

import numpy as np
import cv2
from threading import Event

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge

from record3d import Record3DStream


class Record3DFoundationPoseNode(Node):
    """Bridges a Record3D iPhone stream into ROS 2 for FoundationPose."""

    DEVICE_TYPE_TRUEDEPTH = 0
    DEVICE_TYPE_LIDAR = 1

    def __init__(self):
        super().__init__('record3d_foundationpose_node')

        # ── ROS parameters ──────────────────────────────────────────────
        self.declare_parameter('device_index', 0)
        self.declare_parameter('frame_id', 'iphone_camera')
        self.declare_parameter('world_frame_id', 'world')

        self.dev_idx: int = (
            self.get_parameter('device_index').get_parameter_value().integer_value
        )
        self.frame_id: str = (
            self.get_parameter('frame_id').get_parameter_value().string_value
        )
        self.world_frame_id: str = (
            self.get_parameter('world_frame_id').get_parameter_value().string_value
        )

        # ── Publishers ──────────────────────────────────────────────────
        self.rgb_pub = self.create_publisher(Image, '/iphone/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/iphone/depth/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/iphone/camera_info', 10)
        self.mask_pub = self.create_publisher(Image, '/iphone/segmentation/mask', 10)

        # ── TF broadcaster (camera pose) ────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Helpers ─────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.event = Event()
        self.session: Record3DStream | None = None

        # ── Connect and start ──────────────────────────────────────────
        self._connect_to_device(self.dev_idx)

        # Use a timer at ~60 Hz to drain frames on the ROS executor thread
        self.create_timer(1.0 / 60.0, self._timer_callback)

        self.get_logger().info(
            f'Record3D → FoundationPose bridge running (device {self.dev_idx})'
        )

    # ------------------------------------------------------------------ #
    #  Record3D callbacks (called from a background thread)               #
    # ------------------------------------------------------------------ #
    def _on_new_frame(self):
        self.event.set()

    def _on_stream_stopped(self):
        self.get_logger().warn('Record3D stream stopped')

    # ------------------------------------------------------------------ #
    #  Device connection                                                   #
    # ------------------------------------------------------------------ #
    def _connect_to_device(self, dev_idx: int):
        self.get_logger().info('Searching for Record3D devices …')
        devs = Record3DStream.get_connected_devices()
        self.get_logger().info(f'{len(devs)} device(s) found')

        for dev in devs:
            self.get_logger().info(f'  ID: {dev.product_id}  UDID: {dev.udid}')

        if len(devs) <= dev_idx:
            raise RuntimeError(
                f'Cannot connect to device #{dev_idx} – only {len(devs)} available.'
            )

        self.session = Record3DStream()
        self.session.on_new_frame = self._on_new_frame
        self.session.on_stream_stopped = self._on_stream_stopped
        self.session.connect(devs[dev_idx])

    # ------------------------------------------------------------------ #
    #  Intrinsic matrix helper                                             #
    # ------------------------------------------------------------------ #
    @staticmethod
    def _intrinsic_mat_from_coeffs(coeffs) -> np.ndarray:
        return np.array([
            [coeffs.fx,       0.0, coeffs.tx],
            [      0.0, coeffs.fy, coeffs.ty],
            [      0.0,       0.0,       1.0],
        ])

    # ------------------------------------------------------------------ #
    #  Main publish loop (runs on the executor thread via timer)           #
    # ------------------------------------------------------------------ #
    def _timer_callback(self):
        if not self.event.is_set():
            return
        self.event.clear()

        # ── Grab data from Record3D ─────────────────────────────────────
        depth = self.session.get_depth_frame()          # float32, metres
        rgb = self.session.get_rgb_frame()              # uint8  RGB
        confidence = self.session.get_confidence_frame()
        intrinsics = self._intrinsic_mat_from_coeffs(
            self.session.get_intrinsic_mat()
        )
        camera_pose = self.session.get_camera_pose()

        # ── Mirror TrueDepth data so it matches LiDAR convention ────────
        is_truedepth = (
            self.session.get_device_type() == self.DEVICE_TYPE_TRUEDEPTH
        )
        if is_truedepth:
            depth = cv2.flip(depth, 1)
            rgb = cv2.flip(rgb, 1)

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # ── Common header (all messages share the same stamp) ───────────
        stamp = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = self.frame_id

        h, w = rgb.shape[:2]

        # ── 1. CameraInfo (/iphone/camera_info) ────────────────────────
        cam_info = CameraInfo()
        cam_info.header = header
        cam_info.height = h
        cam_info.width = w
        cam_info.distortion_model = 'plumb_bob'
        cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Record3D already rectified

        fx = float(intrinsics[0, 0])
        fy = float(intrinsics[1, 1])
        cx = float(intrinsics[0, 2])
        cy = float(intrinsics[1, 2])

        # K – 3×3 row‑major
        cam_info.k = [
            fx,  0.0, cx,
            0.0, fy,  cy,
            0.0, 0.0, 1.0,
        ]

        # R – identity (mono camera)
        cam_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        # P – projection matrix (no Tx/Ty for mono)
        cam_info.p = [
            fx,  0.0, cx,  0.0,
            0.0, fy,  cy,  0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        cam_info.binning_x = 0
        cam_info.binning_y = 0

        self.camera_info_pub.publish(cam_info)

        # ── 2. RGB image (/iphone/color/image_raw) ─────────────────────
        rgb_msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        rgb_msg.header = header
        self.rgb_pub.publish(rgb_msg)

        # ── 3. Depth image (/iphone/depth/image_raw) ───────────────────
        depth_msg = self.bridge.cv2_to_imgmsg(
            depth.astype(np.float32), encoding='32FC1'
        )
        depth_msg.header = header
        self.depth_pub.publish(depth_msg)

        # ── 4. Segmentation mask placeholder (/iphone/segmentation/mask)
        #    Replace this with your actual segmentation pipeline output.
        #    A full‑white mask (255) treats the entire image as foreground.
        mask = np.full((h, w), 255, dtype=np.uint8)
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = header
        self.mask_pub.publish(mask_msg)

        # ── 5. TF: world → iphone_camera ───────────────────────────────
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.world_frame_id
        t.child_frame_id = self.frame_id
        t.transform.translation.x = float(camera_pose.tx)
        t.transform.translation.y = float(camera_pose.ty)
        t.transform.translation.z = float(camera_pose.tz)
        t.transform.rotation.x = float(camera_pose.qx)
        t.transform.rotation.y = float(camera_pose.qy)
        t.transform.rotation.z = float(camera_pose.qz)
        t.transform.rotation.w = float(camera_pose.qw)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Record3DFoundationPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()