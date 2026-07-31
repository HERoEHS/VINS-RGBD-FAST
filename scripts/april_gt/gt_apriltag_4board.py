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
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener


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

# center = 보드 그리드 중심의 월드 좌표. PnP 월드 점 생성과 정적 TF 발행의 단일 출처다.
#   z=0.2425는 지표면에서 보드 중심까지의 실측(0.24~0.245) 중간값. 이전 값 0.40은 근거 불명이었다.
#   보드 전부가 같은 z를 쓰므로, 이 정정은 월드 점 집합을 통째로 평행이동시킨다 →
#   PnP가 푸는 카메라 포즈도 z로만 -0.1575 m 이동하고 x/y/yaw는 불변이다.
#   (그래서 기존 GT로 낸 x/y/yaw 기반 결과는 재측정 없이 유효하다.)
#   ※ AR4는 --allowed-boards에서 빠져 현재 미사용이고 높이를 따로 실측하지 않았다.
#      일관성 때문에 같은 값을 넣었으니 활성화 전에 반드시 확인할 것.
BOARDS = [
    {"name": "AR1", "first_id": 0,   "center": np.array([3.25,  0.00, 0.2425]), "R": R_X_WALL},
    {"name": "AR2", "first_id": 36,  "center": np.array([3.25, -0.83, 0.2425]), "R": R_X_WALL},
    {"name": "AR3", "first_id": 72,  "center": np.array([0.00,  0.35, 0.2425]), "R": R_PY_WALL},
    {"name": "AR4", "first_id": 108, "center": np.array([0.00, -1.25, 0.2425]), "R": R_NY_WALL},
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
    # bytes(msg.data) 를 거치지 않는다 — rclpy의 msg.data는 이미 버퍼 프로토콜을 지원하므로
    #   그 호출은 프레임마다 이미지 전체(640x480=307KB)를 한 번 더 복사할 뿐이다.
    #   mono8 경로는 어차피 아래 .copy()로 연속 메모리를 만들고, 컬러 경로는 cvtColor가
    #   새 배열을 내므로 여기서의 복사는 순수 낭비였다.
    data = np.frombuffer(msg.data, dtype=np.uint8)
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


def quat_to_rot(qx, qy, qz, qw):
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def tfmsg_to_matrix(tf_msg):
    """geometry_msgs/TransformStamped → 4x4 동차행렬.

    tf2의 lookup_transform(target, source)는 'source의 점을 target으로 옮기는' 변환을
    돌려주므로, 반환 행렬은 T_target_source 다 (부모=target, 자식=source).
    """
    t = tf_msg.transform.translation
    r = tf_msg.transform.rotation
    M = np.eye(4)
    M[:3, :3] = quat_to_rot(r.x, r.y, r.z, r.w)
    M[:3, 3] = [t.x, t.y, t.z]
    return M


def invert_se3(M):
    Mi = np.eye(4)
    Mi[:3, :3] = M[:3, :3].T
    Mi[:3, 3] = -M[:3, :3].T @ M[:3, 3]
    return Mi


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

        # ── 실시간 3자 비교용 (GT / 휠 / VINS를 같은 프레임에서 겹쳐 보기) ──
        #   ① gt/<base_frame> : GT 카메라 포즈를 로봇 base 점으로 옮긴 정적 자식.
        #      GT는 '카메라' 포즈인데 휠·VINS는 'base' 점을 보고하므로, 같은 점으로
        #      맞추지 않으면 회전할 때마다 레버암만큼 차이가 난다(전역 정렬로 안 풀림).
        #   ② <frame_id> → <pin_odom_frame> : 첫 유효 GT 관측에서 한 번만 계산해 고정.
        #      계속 갱신하면 오차가 이 변환에 흡수돼 화면에서 영원히 겹쳐 보인다 —
        #      측정 대상이 사라지므로 반드시 1회 고정이다.
        self.path_max_poses = args.path_max_poses
        self.base_frame = args.base_frame
        self.camera_optical_frame = args.camera_optical_frame
        self.publish_gt_base_tf = args.publish_gt_base_tf
        self.pin_odom_frame = args.pin_odom_frame
        self.gt_base_child = f"gt/{args.base_frame}"

        needs_static = bool(args.publish_board_tf or self.publish_gt_base_tf or self.pin_odom_frame)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self) if needs_static else None
        self._static_tfs = []          # 누적 목록 — 매번 전체를 다시 보낸다(아래 주석 참조)

        needs_tf_listen = bool(self.publish_gt_base_tf or self.pin_odom_frame)
        self.tf_buffer = Buffer() if needs_tf_listen else None
        self.tf_listener = TransformListener(self.tf_buffer, self) if needs_tf_listen else None
        self.T_cam_base = None         # 카메라 광학 → base (URDF 상수, 지연 조회)
        self.odom_pinned = False

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
            self.publish_static_board_tfs()
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

    def send_static(self, transforms):
        """정적 TF를 누적 목록에 넣고 '전체'를 다시 보낸다.

        StaticTransformBroadcaster.sendTransform()의 누적/치환 동작이 배포판마다 달라서,
        나중에 보낸 것이 앞서 보낸 board TF를 지워버릴 수 있다. 자체 목록을 들고 매번
        전부 재발행하면 어느 구현에서도 안전하다.
        """
        if self.static_tf_broadcaster is None:
            return
        self._static_tfs.extend(transforms)
        self.static_tf_broadcaster.sendTransform(list(self._static_tfs))

    def ensure_cam_base_tf(self):
        """카메라 광학 프레임 → base 프레임의 상수 변환을 URDF TF에서 1회 조회.

        체인(base_footprint→base_link→chassis→stereo_camera_link→optical)이 전부 fixed
        조인트라 상수가 보장된다. 다리 관절은 이 체인 밖(base_link→leg→wheel)이라 무관.
        컨트롤러가 늦게 뜨면 조회가 실패하므로 성공할 때까지 프레임마다 재시도한다.
        """
        if self.T_cam_base is not None:
            return True
        if self.tf_buffer is None:
            return False
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.camera_optical_frame, self.base_frame, rclpy.time.Time())
        except Exception as exc:                                   # noqa: BLE001
            self.get_logger().warn(
                f"TF 조회 대기 중: {self.camera_optical_frame} -> {self.base_frame} ({exc})",
                throttle_duration_sec=5.0)
            return False
        self.T_cam_base = tfmsg_to_matrix(tf_msg)
        t = self.T_cam_base[:3, 3]
        self.get_logger().info(
            f"카메라→base 상수 확보: {self.camera_optical_frame} -> {self.base_frame} "
            f"t=({t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f})")
        if self.publish_gt_base_tf:
            self.send_static([make_transform(
                self.get_clock().now().to_msg(), self.child_frame_id, self.gt_base_child,
                self.T_cam_base[:3, 3], rot_to_quat(self.T_cam_base[:3, :3]))])
            self.get_logger().info(
                f"published static TF: {self.child_frame_id} -> {self.gt_base_child}")
        return True

    def try_pin_odom(self, T_wc):
        """첫 유효 GT 관측에서 <frame_id> → <odom> 을 1회 고정.

            T_W_O = (T_W_C · T_C_B) · T_O_B⁻¹
        이렇게 두면 고정 시점에 휠의 base와 GT의 base가 정확히 겹치고, 이후 벌어지는
        양이 곧 휠 오도메트리의 누적 오차다.
        """
        if self.odom_pinned or not self.pin_odom_frame or self.T_cam_base is None:
            return
        try:
            tf_ob = self.tf_buffer.lookup_transform(
                self.pin_odom_frame, self.base_frame, rclpy.time.Time())
        except Exception as exc:                                   # noqa: BLE001
            self.get_logger().warn(
                f"핀 대기 중: {self.pin_odom_frame} -> {self.base_frame} ({exc})",
                throttle_duration_sec=5.0)
            return
        T_w_b = T_wc @ self.T_cam_base
        T_w_o = T_w_b @ invert_se3(tfmsg_to_matrix(tf_ob))
        self.send_static([make_transform(
            self.get_clock().now().to_msg(), self.frame_id, self.pin_odom_frame,
            T_w_o[:3, 3], rot_to_quat(T_w_o[:3, :3]))])
        self.odom_pinned = True
        t = T_w_o[:3, 3]
        self.get_logger().info(
            f"odom 고정 완료: {self.frame_id} -> {self.pin_odom_frame} "
            f"t=({t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}) — 이후 벌어지는 양이 휠 누적 오차다")

    def publish_static_board_tfs(self):
        # 발행 지점 = 보드 중심(center).
        #   이전에는 그리드 좌하단 모서리(center + R @ [-grid_half, -grid_half, 0])에 발행해서
        #   RViz의 *_board 프레임이 실제 보드 중심에서 grid_half(0.09 m)만큼 어긋나 보였다.
        #   이 함수는 시각화 전용이다 — PnP 월드 점은 board_local_to_world()가 따로 만들고
        #   그쪽은 여전히 모서리 기준이므로 GT 값은 이 변경에 영향받지 않는다.
        stamp = self.get_clock().now().to_msg()
        transforms = []
        active_names = {board["name"] for board in self.board_world_objs}
        for board in BOARDS:
            if board["name"] not in active_names:
                continue
            transforms.append(
                make_transform(
                    stamp,
                    self.frame_id,
                    f"{board['name']}_board",
                    board["center"],
                    rot_to_quat(board["R"]),
                )
            )
        self.send_static(transforms)
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
        # Path는 매번 '누적 전체'를 직렬화해 내보내므로 길이에 비례해 비용이 무한히 자란다.
        #   시각화용이므로 최근 구간만 유지한다 — 분석용 전체 궤적은 TUM 파일에 그대로 남는다.
        if self.path_max_poses > 0 and len(self.path.poses) > self.path_max_poses:
            del self.path.poses[:len(self.path.poses) - self.path_max_poses]
        self.path_pub.publish(self.path)

        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(
                make_transform(stamp, self.frame_id, self.child_frame_id, np.array([tx, ty, tz]), quat_xyzw)
            )

        # 유효 포즈가 나온 뒤에야 상수 조회/핀이 의미를 갖는다 (핀은 T_wc를 쓰므로).
        if self.publish_gt_base_tf or self.pin_odom_frame:
            if self.ensure_cam_base_tf():
                self.try_pin_odom(T_wc)

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
    parser.add_argument("--base-frame", default="base_footprint",
                        help="robot frame to project GT onto (compared against wheel/VINS)")
    parser.add_argument("--camera-optical-frame", default="stereo_camera_left_optical_frame",
                        help="URDF frame corresponding to the PnP camera (OpenCV optical convention)")
    parser.add_argument("--publish-gt-base-tf", action="store_true",
                        help="publish static <child-frame-id> -> gt/<base-frame> so GT is comparable "
                             "at the same physical point as wheel/VINS")
    parser.add_argument("--pin-odom-frame", default="",
                        help="e.g. 'odom'. On the first valid GT pose, publish a ONE-SHOT static "
                             "<frame-id> -> <odom> so both start coincident; later divergence is "
                             "the wheel odometry error. Empty = disabled")
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
    parser.add_argument("--cv-threads", type=int, default=2,
                        help="cv2.setNumThreads(). 0=OpenCV default (grabs every core). "
                             "This is a ~15fps job on a saturated board, so capping it keeps "
                             "cores for VINS and the control loop")
    parser.add_argument("--path-max-poses", type=int, default=2000,
                        help="cap on published Path length (visualization only; the TUM file "
                             "always keeps the full trajectory). 0 = unbounded")
    args = parser.parse_args()

    # OpenCV 스레드 상한. 기본값 그대로 두면 detectMarkers/PnP가 코어를 8개까지 잡아
    #   (실측: 메인 55% + 워커 7개 41% = 123% CPU) 이미 포화된 보드에서 VINS·제어 루프의
    #   CPU를 뺏는다. 이 노드는 15fps 입력 중 ~6개만 유효 포즈로 채택하므로 여유가 있다.
    if args.cv_threads > 0:
        cv2.setNumThreads(args.cv_threads)

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
