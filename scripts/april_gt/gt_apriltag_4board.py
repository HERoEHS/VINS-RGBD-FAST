 #!/usr/bin/env python3
# AprilTag 4-board GT extractor from a ROS 2 image topic.
#
# This script does not open or replay rosbag files. Start the camera publisher or
# run `ros2 bag play ...` separately, then this node subscribes to the image topic
# and writes a world-frame TUM trajectory as detections arrive.

# 명령어: 
# python3 /home/edie/ros2_ws/src/edie9/edie_localization/VINS-RGBD-FAST/scripts/april_gt/gt_apriltag_4board.py   --K 397.64,397.64,339.36,270.85   --tag-size 0.024   --allowed-boards AR1,AR2,AR3   --frame-id map   --child-frame-id apriltag_gt_camera   --gt-odom-topic /apriltag_gt/odom   --gt-path-topic /apriltag_gt/path   --publish-tf   --publish-board-tf   --output ~/ros2_ws/bag/live_apriltag_gt.tum

import argparse
import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


DEFAULT_GRID_ROWS = 6
DEFAULT_GRID_COLS = 6
DEFAULT_TAG_SIZE = 0.024
DEFAULT_TAG_SPACING = 0.3
DEFAULT_IMAGE_TOPIC = "/edie/sensors/camera/left/image_gray"

RECT_CORNERS = np.array([
    [0.00,  0.00, 0.0],
    [2.95,  0.00, 0.0],
    [2.95, -0.90, 0.0],
    [0.00, -0.90, 0.0],
], dtype=np.float64)

# R_world_board columns = [+X_board, +Y_board, +Z_board] in world frame.
# All boards: +Y_board = world +Z. +Z_board = board normal.
R_X_WALL = np.array([[ 0.0, 0.0, -1.0],
                     [-1.0, 0.0,  0.0],
                     [ 0.0, 1.0,  0.0]], dtype=np.float64)
R_PY_WALL = np.array([[1.0, 0.0,  0.0],
                      [0.0, 0.0, -1.0],
                      [0.0, 1.0,  0.0]], dtype=np.float64)
R_NY_WALL = np.array([[-1.0, 0.0, 0.0],
                      [ 0.0, 0.0, 1.0],
                      [ 0.0, 1.0, 0.0]], dtype=np.float64)

BOARDS = [
    {"name": "AR1", "first_id": 0,   "center": np.array([3.25,  0.00, 0.40]), "R": R_X_WALL},
    {"name": "AR2", "first_id": 36,  "center": np.array([3.25, -0.83, 0.40]), "R": R_X_WALL},
    {"name": "AR3", "first_id": 72,  "center": np.array([0.00,  0.35, 0.40]), "R": R_PY_WALL},
    {"name": "AR4", "first_id": 108, "center": np.array([0.00, -1.25, 0.40]), "R": R_NY_WALL},
]

BOARD_NAMES = {board["name"] for board in BOARDS}


def default_output_path(prefix):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.expanduser(f"~/ros2_ws/bag/{prefix}_{stamp}.tum")


def build_board_local_obj_points(rows, cols, tag_size, tag_spacing, first_id=0):
    step = tag_size * (1.0 + tag_spacing)
    obj = {}
    for row in range(rows):
        for col in range(cols):
            tag_id = first_id + row * cols + col
            x0 = col * step
            y0 = row * step
            obj[tag_id] = np.array([
                [x0,            y0 + tag_size, 0.0],
                [x0 + tag_size, y0 + tag_size, 0.0],
                [x0 + tag_size, y0,            0.0],
                [x0,            y0,            0.0],
            ], dtype=np.float64)
    return obj


def board_local_to_world(local_obj, R_world_board, center_world, grid_half):
    origin_world = center_world + R_world_board @ np.array([-grid_half, -grid_half, 0.0])
    return {tag_id: (R_world_board @ pts.T).T + origin_world for tag_id, pts in local_obj.items()}


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


def marker_area(corner):
    pts = corner.reshape(4, 2).astype(np.float32)
    return abs(float(cv2.contourArea(pts)))


def dedupe_detections(ids, corners):
    """Keep the largest marker when OpenCV reports the same tag ID more than once."""
    best = {}
    for tag_id, corner in zip(ids.flatten(), corners):
        tag_id = int(tag_id)
        area = marker_area(corner)
        if tag_id not in best or area > best[tag_id][0]:
            best[tag_id] = (area, corner)
    kept_ids = np.array(sorted(best.keys()), dtype=np.int32)
    kept_corners = [best[tag_id][1] for tag_id in kept_ids]
    return kept_ids, kept_corners


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


def try_pnp_for_board(world_obj_dict, detected_ids, detected_corners, K, D, min_tags, ransac_thresh_px):
    obj_pts = []
    img_pts = []
    for tag_id, corner in zip(detected_ids, detected_corners):
        tag_id = int(tag_id)
        if tag_id in world_obj_dict:
            obj_pts.append(world_obj_dict[tag_id])
            img_pts.append(corner.reshape(4, 2))
    if len(obj_pts) < min_tags:
        return None

    obj_np = np.concatenate(obj_pts, axis=0).astype(np.float64)
    img_np = np.concatenate(img_pts, axis=0).astype(np.float64)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        obj_np,
        img_np,
        K,
        D,
        reprojectionError=ransac_thresh_px,
        iterationsCount=100,
        confidence=0.99,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok or inliers is None or len(inliers) < min_tags * 4:
        return None

    obj_in = obj_np[inliers.flatten()]
    img_in = img_np[inliers.flatten()]
    cv2.solvePnPRefineLM(obj_in, img_in, K, D, rvec, tvec)
    proj, _ = cv2.projectPoints(obj_in, rvec, tvec, K, D)
    err = float(np.linalg.norm(proj.reshape(-1, 2) - img_in, axis=1).mean())

    R_cw, _ = cv2.Rodrigues(rvec)
    R_wc = R_cw.T
    t_wc = -R_wc @ tvec.flatten()
    T_wc = np.eye(4)
    T_wc[:3, :3] = R_wc
    T_wc[:3, 3] = t_wc
    return T_wc, int(len(inliers)), err


def dist_point_to_segment(p, a, b):
    ab = b - a
    t = np.dot(p - a, ab) / np.dot(ab, ab)
    t = np.clip(t, 0.0, 1.0)
    return np.linalg.norm((a + t * ab) - p)


def parse_board_list(value):
    if not value:
        return None
    boards = {item.strip().upper() for item in value.split(",") if item.strip()}
    unknown = boards - BOARD_NAMES
    if unknown:
        raise ValueError(f"unknown board name(s): {', '.join(sorted(unknown))}")
    return boards


def make_transform(stamp, parent_frame, child_frame, translation, quat_xyzw):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = float(translation[0])
    transform.transform.translation.y = float(translation[1])
    transform.transform.translation.z = float(translation[2])
    transform.transform.rotation.x = float(quat_xyzw[0])
    transform.transform.rotation.y = float(quat_xyzw[1])
    transform.transform.rotation.z = float(quat_xyzw[2])
    transform.transform.rotation.w = float(quat_xyzw[3])
    return transform


class FourBoardGtNode(Node):
    def __init__(self, args):
        super().__init__("apriltag_4board_gt_extractor")
        fx, fy, cx, cy = [float(value) for value in args.K.split(",")]
        self.K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        self.D = np.zeros(5, dtype=np.float64)
        self.detector = make_detector()
        self.min_tags = args.min_tags
        self.ransac_thresh = args.ransac_thresh
        self.max_reproj = args.max_reproj
        self.rect_prior_weight = args.rect_prior_weight
        self.jump_prior_weight = args.jump_prior_weight
        self.jump_clip = args.jump_clip
        self.force_board = args.force_board
        self.allowed_boards = parse_board_list(args.allowed_boards)
        self.last_ar = None
        self.last_position = None
        self.rect_segments = [(RECT_CORNERS[i], RECT_CORNERS[(i + 1) % 4]) for i in range(4)]

        step = args.tag_size * (1.0 + args.tag_spacing)
        grid_span = (max(args.rows, args.cols) - 1) * step + args.tag_size
        grid_half = grid_span / 2.0
        self.board_world_objs = []
        for b in BOARDS:
            if self.force_board and b["name"] != self.force_board:
                continue
            if self.allowed_boards and b["name"] not in self.allowed_boards:
                continue
            local_obj = build_board_local_obj_points(
                args.rows, args.cols, args.tag_size, args.tag_spacing, first_id=b["first_id"]
            )
            self.board_world_objs.append(
                {"name": b["name"], "obj": board_local_to_world(local_obj, b["R"], b["center"], grid_half)}
            )
        if not self.board_world_objs:
            raise ValueError("no active board hypotheses after applying --force-board/--allowed-boards")

        self.output = os.path.expanduser(args.output)
        stem = os.path.splitext(self.output)[0]
        self.summary_output = stem + "_summary.txt"
        self.debug_output = stem + "_debug.csv"
        os.makedirs(os.path.dirname(self.output) or ".", exist_ok=True)
        self.out = open(self.output, "w")
        self.debug = open(self.debug_output, "w")
        self.debug.write("t_sec,ar,n_inliers,mean_reproj_px,rect_dist_m,jump_m,score,tx,ty,tz\n")
        self.frame_id = args.frame_id
        self.child_frame_id = args.child_frame_id
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.odom_pub = self.create_publisher(Odometry, args.gt_odom_topic, 10)
        self.path_pub = self.create_publisher(Path, args.gt_path_topic, 10)
        self.publish_tf = args.publish_tf
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.static_tf_broadcaster = StaticTransformBroadcaster(self) if args.publish_board_tf else None

        self.n_frames = 0
        self.n_no_detect = 0
        self.n_no_valid = 0
        self.n_high_err = 0
        self.poses = []
        self.per_board_count = {b["name"]: 0 for b in BOARDS}

        self.create_subscription(Image, args.image_topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f"subscribed: {args.image_topic}")
        self.get_logger().info(f"writing TUM: {self.output}")
        self.get_logger().info(f"publishing: {args.gt_odom_topic} (Odometry), {args.gt_path_topic} (Path), frame_id={self.frame_id}")
        if self.publish_tf:
            self.get_logger().info(f"publishing dynamic TF: {self.frame_id} -> {self.child_frame_id}")
        if args.publish_board_tf:
            self.publish_static_board_tfs(args.rows, args.cols, args.tag_size, args.tag_spacing)
        id_ranges = ", ".join(
            f"{b['name']}={b['first_id']}-{b['first_id'] + args.rows * args.cols - 1}" for b in BOARDS
        )
        self.get_logger().info(f"expected unique board ID ranges: {id_ranges}")
        self.get_logger().info(
            f"grid={args.rows}x{args.cols}, tag_size={args.tag_size:.3f}m, spacing={args.tag_spacing}, span={grid_span:.3f}m"
        )
        self.get_logger().info("active board hypotheses: " + ", ".join(b["name"] for b in self.board_world_objs))
        self.get_logger().info(
            "board selection score = reproj_px "
            f"+ {self.rect_prior_weight:g}*rect_dist_m "
            f"+ {self.jump_prior_weight:g}*min(jump_m,{self.jump_clip:g})"
        )

    def publish_static_board_tfs(self, rows, cols, tag_size, tag_spacing):
        step = tag_size * (1.0 + tag_spacing)
        grid_span = (max(rows, cols) - 1) * step + tag_size
        grid_half = grid_span / 2.0
        stamp = self.get_clock().now().to_msg()
        transforms = []
        active_names = {board["name"] for board in self.board_world_objs}
        for board in BOARDS:
            if board["name"] not in active_names:
                continue
            board_origin = board["center"] + board["R"] @ np.array([-grid_half, -grid_half, 0.0])
            transforms.append(
                make_transform(
                    stamp,
                    self.frame_id,
                    f"{board['name']}_board",
                    board_origin,
                    rot_to_quat(board["R"]),
                )
            )
        self.static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info(
            "published static board TFs: " + ", ".join(transform.child_frame_id for transform in transforms)
        )

    def rect_distance(self, T_wc):
        pos = T_wc[:3, 3]
        p_xy = np.array([pos[0], pos[1], 0.0])
        return min(dist_point_to_segment(p_xy, a, b) for a, b in self.rect_segments)

    def candidate_score(self, err, T_wc):
        rect_dist = self.rect_distance(T_wc)
        if self.last_position is None:
            jump = 0.0
        else:
            jump = float(np.linalg.norm(T_wc[:3, 3] - self.last_position))
        score = (
            err
            + self.rect_prior_weight * rect_dist
            + self.jump_prior_weight * min(jump, self.jump_clip)
        )
        return score, rect_dist, jump

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
            return
        detected_ids, detected_corners = dedupe_detections(ids, corners)

        candidates = []
        for board in self.board_world_objs:
            res = try_pnp_for_board(
                board["obj"], detected_ids, detected_corners, self.K, self.D, self.min_tags, self.ransac_thresh
            )
            if res is not None:
                T_wc, n_in, err = res
                score, rect_dist, jump = self.candidate_score(err, T_wc)
                candidates.append((score, err, rect_dist, jump, board["name"], T_wc, n_in))

        if not candidates:
            self.n_no_valid += 1
            return

        candidates.sort(key=lambda item: item[0])
        best_score, best_err, best_rect_dist, best_jump, best_name, best_T, best_in = candidates[0]

        if best_err > self.max_reproj:
            self.n_high_err += 1
            return

        self.write_pose(msg, best_T, best_name, best_in, best_err, best_rect_dist, best_jump, best_score)

    def write_pose(self, msg, T_wc, board_name, n_inliers, err_px, rect_dist, jump, score):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        tx, ty, tz = T_wc[:3, 3]
        qx, qy, qz, qw = rot_to_quat(T_wc[:3, :3])
        self.out.write(
            f"{t_sec:.9f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n"
        )
        self.debug.write(
            f"{t_sec:.6f},{board_name},{n_inliers},{err_px:.3f},"
            f"{rect_dist:.4f},{jump:.4f},{score:.4f},{tx:.4f},{ty:.4f},{tz:.4f}\n"
        )
        self.poses.append((t_sec, T_wc, board_name, n_inliers, err_px))
        self.per_board_count[board_name] += 1
        self.last_ar = board_name
        self.last_position = T_wc[:3, 3].copy()
        self.publish_pose(msg, T_wc, (qx, qy, qz, qw))
        if len(self.poses) % 30 == 0:
            self.out.flush()
            self.debug.flush()
            self.get_logger().info(f"GT poses={len(self.poses)}, frames={self.n_frames}, last={board_name}")

    def publish_pose(self, image_msg, T_wc, quat_xyzw):
        stamp = image_msg.header.stamp
        tx, ty, tz = T_wc[:3, 3]
        qx, qy, qz, qw = quat_xyzw

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = float(tx)
        odom.pose.pose.position.y = float(ty)
        odom.pose.pose.position.z = float(tz)
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(
                make_transform(stamp, self.frame_id, self.child_frame_id, np.array([tx, ty, tz]), quat_xyzw)
            )

    def finish(self):
        self.out.flush()
        self.debug.flush()
        self.out.close()
        self.debug.close()
        n_detect = len(self.poses)
        if n_detect == 0:
            self.get_logger().warning(
                f"no valid GT poses. frames={self.n_frames}, no_detect={self.n_no_detect}, "
                f"no_valid={self.n_no_valid}, high_reproj={self.n_high_err}"
            )
            return

        positions = np.array([T[:3, 3] for _, T, *_ in self.poses])
        total_dist = float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()) if n_detect > 1 else 0.0
        duration = self.poses[-1][0] - self.poses[0][0]
        drift_pos = float(np.linalg.norm(positions[-1] - positions[0]))
        R_rel = self.poses[-1][1][:3, :3] @ self.poses[0][1][:3, :3].T
        cos_ang = np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0)
        drift_ang_deg = float(np.degrees(np.arccos(cos_ang)))
        rect_segments = [(RECT_CORNERS[i], RECT_CORNERS[(i + 1) % 4]) for i in range(4)]
        rect_dists = np.array([
            min(dist_point_to_segment(np.array([p[0], p[1], 0.0]), a, b) for a, b in rect_segments)
            for p in positions
        ])
        err_arr = np.array([err for *_, err in self.poses])
        inlier_arr = np.array([n_in for *_, n_in, _ in self.poses])
        det_rate = 100.0 * n_detect / max(self.n_frames, 1)

        with open(self.summary_output, "w") as summary:
            summary.write("mode: topic_subscriber\n")
            summary.write(f"output_tum: {self.output}\n")
            summary.write(f"output_debug_csv: {self.debug_output}\n")
            summary.write(f"rect_prior_weight: {self.rect_prior_weight}\n")
            summary.write(f"jump_prior_weight: {self.jump_prior_weight}\n")
            summary.write(f"jump_clip_m: {self.jump_clip}\n")
            summary.write(f"frames_total: {self.n_frames}\n")
            summary.write(f"frames_with_gt: {n_detect}\n")
            summary.write(f"detection_rate_pct: {det_rate:.2f}\n")
            summary.write(f"gt_duration_sec: {duration:.3f}\n")
            summary.write(f"total_distance_m: {total_dist:.4f}\n")
            summary.write("rect_perimeter_expected_m: 7.7000\n")
            summary.write(f"rect_dist_mean_m: {rect_dists.mean():.4f}\n")
            summary.write(f"rect_dist_p95_m: {np.percentile(rect_dists, 95):.4f}\n")
            summary.write(f"rect_dist_max_m: {rect_dists.max():.4f}\n")
            summary.write(f"endpoint_drift_pos_m: {drift_pos:.4f}\n")
            summary.write(f"endpoint_drift_rot_deg: {drift_ang_deg:.4f}\n")
            summary.write(f"reproj_err_mean_px: {err_arr.mean():.3f}\n")
            summary.write(f"reproj_err_max_px: {err_arr.max():.3f}\n")
            summary.write(f"inliers_mean_corners: {inlier_arr.mean():.2f}\n")
            for name in ("AR1", "AR2", "AR3", "AR4"):
                summary.write(f"board_{name}_count: {self.per_board_count[name]}\n")

        self.get_logger().info(f"saved: {self.output} ({n_detect} poses)")
        self.get_logger().info(f"saved: {self.summary_output}")
        self.get_logger().info(f"saved: {self.debug_output}")


def main():
    parser = argparse.ArgumentParser(description="4-board AprilTag GT extractor from a ROS 2 image topic")
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--K", required=True, help="rectified intrinsics: fx,fy,cx,cy")
    parser.add_argument("--output", default=default_output_path("apriltag_4b_gt"), help="output TUM path")
    parser.add_argument("--gt-odom-topic", default="/apriltag_gt/odom", help="published AprilTag GT Odometry topic")
    parser.add_argument("--gt-path-topic", default="/apriltag_gt/path", help="published AprilTag GT Path topic")
    parser.add_argument("--frame-id", default="april_gt_world", help="frame_id for published GT odom/path")
    parser.add_argument("--child-frame-id", default="apriltag_camera", help="child_frame_id for published GT odometry")
    parser.add_argument("--publish-tf", action="store_true", help="publish dynamic TF from --frame-id to --child-frame-id")
    parser.add_argument("--publish-board-tf", action="store_true", help="publish static TFs for active board origins")
    parser.add_argument("--tag-size", type=float, default=DEFAULT_TAG_SIZE)
    parser.add_argument("--tag-spacing", type=float, default=DEFAULT_TAG_SPACING)
    parser.add_argument("--rows", type=int, default=DEFAULT_GRID_ROWS)
    parser.add_argument("--cols", type=int, default=DEFAULT_GRID_COLS)
    parser.add_argument("--min-tags", type=int, default=4)
    parser.add_argument("--ransac-thresh", type=float, default=4.0)
    parser.add_argument("--max-reproj", type=float, default=3.0)
    parser.add_argument(
        "--rect-prior-weight",
        type=float,
        default=20.0,
        help="candidate score penalty in px per meter away from the known rectangle path",
    )
    parser.add_argument(
        "--jump-prior-weight",
        type=float,
        default=3.0,
        help="candidate score penalty in px per meter of frame-to-frame pose jump",
    )
    parser.add_argument(
        "--jump-clip",
        type=float,
        default=2.0,
        help="clip jump penalty at this many meters so long occlusions can recover",
    )
    parser.add_argument(
        "--force-board",
        choices=sorted(BOARD_NAMES),
        help="use only one known board hypothesis; useful when extracting a segment facing one board",
    )
    parser.add_argument(
        "--allowed-boards",
        help="comma-separated board hypotheses to allow, e.g. AR3,AR4; useful for segment tests",
    )
    args = parser.parse_args()

    rclpy.init()
    node = FourBoardGtNode(args)
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
