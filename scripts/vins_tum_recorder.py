#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# VINS 추정 궤적 → TUM 파일 기록기 (AprilTag GT와 정량 비교용)
#
# [무엇을 하나]
#   /vins_estimator/camera_pose (PoseStamped) 또는 /vins_estimator/odometry (Odometry)를
#   구독해서, 들어오는 포즈를 GT 스크립트와 "완전히 동일한 TUM 포맷"으로 한 줄씩 저장한다.
#   TUM 한 줄 = "t tx ty tz qx qy qz qw"  (시각 단위: 초, 쿼터니언 xyzw)
#
# [왜 camera_pose 기본인가]
#   GT(apriltag_gt_camera)는 '카메라' 포즈다. 추정도 '카메라'(/vins_estimator/camera_pose)로 맞추면
#   같은 물리점이라 evo -a(SE3 정렬) 한 번으로 두 월드 원점 차이만 풀려 깔끔하게 비교된다.
#   (body로 비교하려면 카메라-IMU extrinsic 보정이 추가로 필요 → 그땐 --topic /vins_estimator/odometry --msg-type odom)
#
# [시각(stamp)]
#   TUM stamp는 메시지 header.stamp(=bag 재생 시각)를 그대로 쓰므로, GT와 같은 bag 시계 위에 찍힌다.
#   use_sim_time은 노드 now()에만 영향 → TUM 정합엔 영향 없지만 일관성 위해 켜는 걸 권장.
#
# [사용법]
#   # 터미널1: ros2 bag play ... --clock 400 ...
#   # 터미널2: ros2 launch vins_estimator edie_vslam.launch.py use_sim_time:=true
#   # 터미널3 (이 노드):
#   python3 ~/ros2_ws/src/edie9/edie_localization/VINS-RGBD-FAST/scripts/vins_tum_recorder.py \
#   --topic /vins_estimator/camera_pose --msg-type pose \
#   --output ~/ros2_ws/bag/vins_camera.tum \
#   --ros-args -p use_sim_time:=true
#   # bag 재생이 끝나면 Ctrl+C → 파일 flush/close 후 요약 출력.
#
# [그 다음 evo 비교 (카메라 기준)]
#   evo_ape  tum ~/ros2_ws/bag/live_apriltag_gt.tum ~/ros2_ws/bag/vins_camera.tum -a  --t_max_diff 0.02 --plot --plot_mode xy
#   evo_rpe  tum ~/ros2_ws/bag/live_apriltag_gt.tum ~/ros2_ws/bag/vins_camera.tum -a  --delta 1 --delta_unit m
#   evo_traj tum ~/ros2_ws/bag/vins_camera.tum --ref ~/ros2_ws/bag/live_apriltag_gt.tum -a -p
#   # scale 진단(메트릭이 맞는지): -a 대신 -as(Sim3) 한 번 더 → 출력의 scale correction 확인

import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def parse_args(argv):
    p = argparse.ArgumentParser(description="VINS pose → TUM trajectory recorder")
    p.add_argument("--topic", default="/vins_estimator/camera_pose",
                   help="구독할 토픽 (기본: /vins_estimator/camera_pose)")
    p.add_argument("--msg-type", choices=["pose", "odom"], default="pose",
                   help="pose=PoseStamped, odom=nav_msgs/Odometry (기본: pose)")
    p.add_argument("--output", default="~/ros2_ws/bag/vins_camera.tum",
                   help="저장할 TUM 파일 경로")
    # rclpy 파라미터(-p use_sim_time:=true 등)는 --ros-args 뒤로 넘어가므로 여기선 무시한다.
    known, _ = p.parse_known_args(argv)
    return known


class TumRecorder(Node):
    def __init__(self, args):
        super().__init__("vins_tum_recorder")
        self.output = os.path.expanduser(args.output)
        os.makedirs(os.path.dirname(self.output) or ".", exist_ok=True)
        self.out = open(self.output, "w")
        self.count = 0

        # VINS 퍼블리셔는 기본 RELIABLE/KEEP_LAST → 동일하게 맞춰 구독해야 누락이 없다.
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        if args.msg_type == "pose":
            self.create_subscription(PoseStamped, args.topic, self.on_pose, qos)
        else:
            self.create_subscription(Odometry, args.topic, self.on_odom, qos)

        self.get_logger().info(f"subscribed: {args.topic} ({args.msg_type})")
        self.get_logger().info(f"writing TUM: {self.output}")

    def _write(self, stamp, px, py, pz, qx, qy, qz, qw):
        t = stamp.sec + stamp.nanosec * 1e-9
        # GT 스크립트와 동일한 포맷/자릿수 (evo 호환)
        self.out.write(f"{t:.9f} {px:.6f} {py:.6f} {pz:.6f} "
                       f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
        self.count += 1
        if self.count % 200 == 0:
            self.out.flush()
            self.get_logger().info(f"recorded poses = {self.count}")

    def on_pose(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        self._write(msg.header.stamp, p.x, p.y, p.z, o.x, o.y, o.z, o.w)

    def on_odom(self, msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        self._write(msg.header.stamp, p.x, p.y, p.z, o.x, o.y, o.z, o.w)

    def finish(self):
        self.out.flush()
        self.out.close()
        self.get_logger().info(f"DONE. total poses = {self.count} → {self.output}")
        if self.count == 0:
            self.get_logger().warning(
                "포즈 0개. 토픽명/메시지타입/QoS 또는 VINS 초기화 여부를 확인하세요.")


def main():
    args = parse_args(sys.argv[1:])
    rclpy.init(args=sys.argv)
    node = TumRecorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
