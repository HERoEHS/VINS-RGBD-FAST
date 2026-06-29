#!/usr/bin/env python3
# AprilTag single-board GT extractor from a ROS 2 image topic.
#
# This script does not open or replay rosbag files. Start the camera publisher or
# run `ros2 bag play ...` separately, then this node subscribes to the image topic
# and writes TUM trajectory lines as detections arrive.

import argparse
import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


DEFAULT_GRID_ROWS = 6
DEFAULT_GRID_COLS = 6
DEFAULT_TAG_SIZE = 0.024
DEFAULT_TAG_SPACING = 0.3
DEFAULT_IMAGE_TOPIC = "/edie/sensors/camera/left/image_gray"


def default_output_path(prefix):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.expanduser(f"~/ros2_ws/bag/{prefix}_{stamp}.tum")


def build_board_object_points(rows, cols, tag_size, tag_spacing):
    """Return tag_id -> 4 corner points in board frame. Corner order: TL, TR, BR, BL."""
    step = tag_size * (1.0 + tag_spacing)
    obj = {}
    for row in range(rows):
        for col in range(cols):
            tag_id = row * cols + col
            x0 = col * step
            y0 = row * step
            obj[tag_id] = np.array([
                [x0,            y0 + tag_size, 0.0],
                [x0 + tag_size, y0 + tag_size, 0.0],
                [x0 + tag_size, y0,            0.0],
                [x0,            y0,            0.0],
            ], dtype=np.float64)
    return obj


def make_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    return cv2.aruco.ArucoDetector(dictionary, params)


def image_to_gray(msg):
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding in ("mono8", "8UC1"):
        frame = data.reshape(msg.height, msg.step)[:, :msg.width]
        return frame.copy()
    if msg.encoding in ("bgr8", "rgb8"):
        frame = data.reshape(msg.height, msg.step // 3, 3)[:, :msg.width, :]
        code = cv2.COLOR_RGB2GRAY if msg.encoding == "rgb8" else cv2.COLOR_BGR2GRAY
        return cv2.cvtColor(frame, code)
    raise ValueError(f"unsupported image encoding: {msg.encoding}")


def rot_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


class SingleBoardGtNode(Node):
    def __init__(self, args):
        super().__init__("apriltag_gt_extractor")
        fx, fy, cx, cy = [float(value) for value in args.K.split(",")]
        self.K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        self.D = np.zeros(5, dtype=np.float64)
        self.obj_pts = build_board_object_points(args.rows, args.cols, args.tag_size, args.tag_spacing)
        self.detector = make_detector()
        self.min_tags = args.min_tags
        self.output = os.path.expanduser(args.output)
        self.summary_output = os.path.splitext(self.output)[0] + "_summary.txt"
        os.makedirs(os.path.dirname(self.output) or ".", exist_ok=True)
        self.out = open(self.output, "w")

        self.n_frames = 0
        self.n_no_detect = 0
        self.n_no_valid = 0
        self.poses = []
        self.tag_counts = []

        self.create_subscription(Image, args.image_topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f"subscribed: {args.image_topic}")
        self.get_logger().info(f"writing TUM: {self.output}")
        self.get_logger().info(
            f"grid={args.rows}x{args.cols}, tag_size={args.tag_size:.3f}m, spacing={args.tag_spacing}"
        )

    def on_image(self, msg):
        self.n_frames += 1
        try:
            gray = image_to_gray(msg)
        except ValueError as exc:
            if self.n_frames == 1:
                self.get_logger().warning(str(exc))
            return

        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            self.n_no_detect += 1
            self.tag_counts.append(0)
            return

        valid_obj = []
        valid_img = []
        for corner, tag_id in zip(corners, ids.flatten()):
            tag_id = int(tag_id)
            if tag_id in self.obj_pts:
                valid_obj.append(self.obj_pts[tag_id])
                valid_img.append(corner.reshape(4, 2))
        self.tag_counts.append(len(valid_obj))
        if len(valid_obj) < self.min_tags:
            self.n_no_valid += 1
            return

        obj_np = np.concatenate(valid_obj, axis=0).astype(np.float64)
        img_np = np.concatenate(valid_img, axis=0).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj_np, img_np, self.K, self.D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            self.n_no_valid += 1
            return

        R_cw, _ = cv2.Rodrigues(rvec)
        R_wc = R_cw.T
        t_wc = -R_wc @ tvec.flatten()
        T_wc = np.eye(4)
        T_wc[:3, :3] = R_wc
        T_wc[:3, 3] = t_wc
        self.write_pose(msg, T_wc)

    def write_pose(self, msg, T_wc):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        tx, ty, tz = T_wc[:3, 3]
        qx, qy, qz, qw = rot_to_quat(T_wc[:3, :3])
        self.out.write(
            f"{t_sec:.9f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n"
        )
        self.poses.append((t_sec, T_wc))
        if len(self.poses) % 30 == 0:
            self.out.flush()
            self.get_logger().info(f"GT poses={len(self.poses)}, frames={self.n_frames}")

    def finish(self):
        self.out.flush()
        self.out.close()
        n_detect = len(self.poses)
        if n_detect == 0:
            self.get_logger().warning(
                f"no valid GT poses. frames={self.n_frames}, no_detect={self.n_no_detect}, no_valid={self.n_no_valid}"
            )
            return

        positions = np.array([T[:3, 3] for _, T in self.poses])
        total_dist = float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()) if n_detect > 1 else 0.0
        duration = self.poses[-1][0] - self.poses[0][0]
        drift_pos = float(np.linalg.norm(positions[-1] - positions[0]))
        tag_counts = np.array(self.tag_counts)
        avg_tags = float(tag_counts[tag_counts > 0].mean()) if (tag_counts > 0).any() else 0.0
        det_rate = 100.0 * n_detect / max(self.n_frames, 1)

        with open(self.summary_output, "w") as summary:
            summary.write("mode: topic_subscriber\n")
            summary.write(f"output_tum: {self.output}\n")
            summary.write(f"frames_total: {self.n_frames}\n")
            summary.write(f"frames_with_gt: {n_detect}\n")
            summary.write(f"detection_rate_pct: {det_rate:.2f}\n")
            summary.write(f"avg_tags_when_seen: {avg_tags:.2f}\n")
            summary.write(f"gt_duration_sec: {duration:.3f}\n")
            summary.write(f"total_distance_m: {total_dist:.4f}\n")
            summary.write(f"endpoint_drift_pos_m: {drift_pos:.4f}\n")

        self.get_logger().info(f"saved: {self.output} ({n_detect} poses)")
        self.get_logger().info(f"saved: {self.summary_output}")


def main():
    parser = argparse.ArgumentParser(description="AprilTag single-board GT extractor from a ROS 2 image topic")
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--K", required=True, help="rectified intrinsics: fx,fy,cx,cy")
    parser.add_argument("--output", default=default_output_path("apriltag_gt"), help="output TUM path")
    parser.add_argument("--tag-size", type=float, default=DEFAULT_TAG_SIZE)
    parser.add_argument("--tag-spacing", type=float, default=DEFAULT_TAG_SPACING)
    parser.add_argument("--rows", type=int, default=DEFAULT_GRID_ROWS)
    parser.add_argument("--cols", type=int, default=DEFAULT_GRID_COLS)
    parser.add_argument("--min-tags", type=int, default=4)
    args = parser.parse_args()

    rclpy.init()
    node = SingleBoardGtNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
