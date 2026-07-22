#!/usr/bin/env python3
"""[SW1-1837] 실로봇 라이브 yaw 누적오차 판별기.

VINS yaw vs gyro 적분 yaw를 실시간 비교해서, 회전 시 뒤처짐이
'출력 지연(무해)'인지 'yaw 누적 오차(실오차)'인지 판별한다.

원리: bag 검증에서 확립된 사실 — 이 로봇 gyro는 GT 대비 -0.11%로 거의
완벽한 기준자다. VINS yaw가 gyro 적분과 벌어진 채 정지 후에도 유지되면
실오차, 정지하면 따라잡으면 지연이다.

사용 (로봇에서, vslam 실행 중):
    python3 yaw_live_check.py
    # 프로토콜: ①15초 정지(gyro bias 자가측정 + BGZ-LOCK 발동 대기)
    #           ②제자리 2바퀴 회전 → 3초 정지 → 출력 읽기 → 반복
판독:
    diff가 정지 시마다 ~0으로 복귀        → 지연 (bgz_lock 무관, 무해)
    diff가 회전량에 비례해 계단식 누적    → 실오차 (회전당 -0.7°=정상 상한,
                                            회전당 -7°급=Bg_z 잠금 미발동 의심)
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

IMU_TOPIC = "/edie/sensor/offset_imu"
VINS_TOPIC = "/vins_estimator/odometry"
BIAS_WINDOW_SEC = 10.0  # 시작 정지 구간에서 gyro z bias 자가측정


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class YawLiveCheck(Node):
    def __init__(self):
        super().__init__("yaw_live_check")
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, IMU_TOPIC, self.imu_cb, qos)
        self.create_subscription(Odometry, VINS_TOPIC, self.vins_cb, qos)
        self.create_timer(1.0, self.report)

        self.t0 = None          # 첫 IMU 수신 시각
        self.last_t = None
        self.bias_samples = []  # 초기 정지 구간 gyro z
        self.bias = None
        self.gyro_yaw = 0.0     # gyro 적분 yaw [rad]
        self.abs_rot = 0.0      # 누적 |회전량| [rad] — 비례 판독용
        self.vins_yaw = None    # unwrap된 VINS yaw [rad]
        self.vins_yaw_raw_prev = None
        self.vins_t = None
        self.imu_t = None

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imu_t = t
        if self.t0 is None:
            self.t0 = t
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0 or dt > 0.5:
            return
        wz = msg.angular_velocity.z
        if t - self.t0 < BIAS_WINDOW_SEC:
            self.bias_samples.append(wz)
            return
        if self.bias is None:
            if not self.bias_samples:
                self.bias = 0.0
            else:
                s = sorted(self.bias_samples)
                self.bias = s[len(s) // 2]  # 중앙값
            self.get_logger().info(
                f"[준비 완료] gyro z bias={self.bias:+.6f} rad/s "
                f"(정지 {BIAS_WINDOW_SEC:.0f}s 중앙값) — 이제 회전 시작")
        w = wz - self.bias
        self.gyro_yaw += w * dt
        self.abs_rot += abs(w) * dt

    def vins_cb(self, msg):
        raw = quat_to_yaw(msg.pose.pose.orientation)
        if self.vins_yaw is None:
            self.vins_yaw = raw
        else:
            d = raw - self.vins_yaw_raw_prev
            while d > math.pi:
                d -= 2.0 * math.pi
            while d < -math.pi:
                d += 2.0 * math.pi
            self.vins_yaw += d
        self.vins_yaw_raw_prev = raw
        self.vins_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def report(self):
        if self.bias is None:
            n = len(self.bias_samples)
            self.get_logger().info(f"[bias 측정 중] 정지 유지... ({n} 샘플)")
            return
        if self.vins_yaw is None:
            self.get_logger().info("[대기] VINS odometry 미수신")
            return
        # VINS 발행이 gyro보다 늦으므로 두 스탬프 차이도 표시(지연 판독 보조)
        lag_ms = (self.imu_t - self.vins_t) * 1e3 if self.vins_t else float("nan")
        deg = 180.0 / math.pi
        diff = (self.vins_yaw - self.gyro_yaw) * deg
        self.get_logger().info(
            f"VINS {self.vins_yaw*deg:+8.2f}°  gyro {self.gyro_yaw*deg:+8.2f}°  "
            f"차이 {diff:+6.2f}°  누적회전 {self.abs_rot*deg:7.1f}°  "
            f"VINS지연 {lag_ms:5.0f}ms")


def main():
    rclpy.init(args=sys.argv)
    node = YawLiveCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
