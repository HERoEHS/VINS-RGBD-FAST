#!/usr/bin/env python3
"""
anchor_ruler_check.py — station_docking 앵커가 "어느 자(ruler)"로 찍혔는지 로봇 위에서 판별

배경 (같은 자 원칙):
  station_docking_node는 odom→station_docking_obs TF(앵커)를 발행한다.
  robot_pose_frame 선택자(3d96e6e) 적용 후에는 앵커가 VINS 자
  (odom→base_vins ∘ base_footprint→optical)로 찍혀야 한다.
  이 스크립트는 배포된 로봇에서 그게 실제로 그런지 실측으로 확인한다.

판별 원리:
  anchor = lookup(odom→station_docking_obs)          # 노드가 찍은 앵커
  후보 자 두 개로 같은 시각의 odom→optical을 계산:
    wheel 자 : lookup(odom→optical)                  # 휠 odometry 경유 전체 트리
    VINS 자  : lookup(odom→base_vins) ∘ lookup(base_footprint→optical)
  inv(후보 체인) @ anchor = T_optical→obs 는
  "앵커를 찍을 때 쓴 자"에 대해서만 시간에 대해 거의 상수다(마커가 고정이므로).
  → 표본들의 위치 표준편차가 작은 쪽 = 실제 사용된 자.

사용법 (로봇에서, HF ON + station_docking 기동 상태):
  1) 먼저 주행/제자리회전으로 두 자(base_vins vs base_footprint)를 5mm 이상 벌린다.
  2) python3 anchor_ruler_check.py [표본수=20]
  기대 출력: "✅ 앵커는 VINS 자(base_vins)"
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros

ODOM = 'odom'
OBS = 'station_docking_obs'
BASE_VINS = 'base_vins'
BASE_FOOT = 'base_footprint'
OPTICAL = 'stereo_camera_left_optical_frame'
SEP_MIN_M = 0.005  # 두 자가 이보다 덜 벌어져 있으면 판별 불가


def quat_to_rot(x, y, z, w):
    """쿼터니언 → 3x3 회전행렬 (tf_transformations 미설치 로봇 대응, numpy만 사용)"""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def tf_to_mat(tf_msg):
    """TransformStamped → 4x4 동차행렬"""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    m = np.eye(4)
    m[0:3, 0:3] = quat_to_rot(q.x, q.y, q.z, q.w)
    m[0:3, 3] = [t.x, t.y, t.z]
    return m


class RulerCheck(Node):
    def __init__(self, n_samples):
        super().__init__('anchor_ruler_check')
        self.n_samples = n_samples
        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        # 표본: (T_optical→obs)의 위치를 자별로 누적
        self.rel_wheel = []   # inv(wheel 체인) @ anchor 의 위치
        self.rel_vins = []    # inv(VINS 체인) @ anchor 의 위치
        self.seps = []        # 두 자 optical 위치 간 거리 (판별 가능성 지표)
        self.last_obs_stamp = None
        self.timer = self.create_timer(0.2, self.tick)

    def tick(self):
        try:
            anchor = self.buffer.lookup_transform(ODOM, OBS, Time())
        except Exception:
            return  # 앵커 아직 없음 (마커 미관측)
        stamp = Time.from_msg(anchor.header.stamp)
        if self.last_obs_stamp is not None and stamp == self.last_obs_stamp:
            return  # 같은 관측의 앵커 재발행 — 스탬프 고정 조회라 완전 중복 표본이므로 스킵
        # ⚠️ 핵심: 후보 체인을 반드시 "앵커가 찍힌 시각(stamp)"으로 조회한다.
        #   latest 로 조회하면 로봇 이동 시 t_img↔now 사이 이동량이 양쪽 후보 모두를
        #   오염시켜 판별이 무너진다 — 08-20 실기(양쪽 산포 170~190mm)에서 실증한 버그.
        try:
            wheel_chain = self.buffer.lookup_transform(ODOM, OPTICAL, stamp)
            odom_vins = self.buffer.lookup_transform(ODOM, BASE_VINS, stamp)
            base_opt = self.buffer.lookup_transform(BASE_FOOT, OPTICAL, stamp)
        except tf2_ros.ExtrapolationException:
            return  # 해당 시각 TF가 아직/이미 버퍼 범위 밖 — 다음 관측에서 재시도
        except Exception as e:
            self.get_logger().warn(f'조회 실패(HF ON? base_vins 발행 중?): {e}')
            return
        self.last_obs_stamp = stamp
        # z 기반 결정 판별용 최신값 보관
        self.last_anchor = anchor
        self.last_vins_z = odom_vins.transform.translation.z

        m_anchor = tf_to_mat(anchor)
        m_wheel = tf_to_mat(wheel_chain)
        m_vins = tf_to_mat(odom_vins) @ tf_to_mat(base_opt)

        # 두 자의 벌어짐 (optical 위치 차)
        sep = float(np.linalg.norm(m_wheel[0:3, 3] - m_vins[0:3, 3]))
        self.seps.append(sep)

        self.rel_wheel.append((np.linalg.inv(m_wheel) @ m_anchor)[0:3, 3].copy())
        self.rel_vins.append((np.linalg.inv(m_vins) @ m_anchor)[0:3, 3].copy())
        k = len(self.rel_wheel)
        self.get_logger().info(f'표본 {k}/{self.n_samples}  두 자 벌어짐={sep*1000:.1f}mm')
        if k >= self.n_samples:
            self.report()
            self.report_z(self.last_anchor, self.last_vins_z)
            raise SystemExit

    def report(self):
        def spread(samples):
            a = np.array(samples)
            return float(np.linalg.norm(a.std(axis=0)))  # 축별 std의 노름 [m]

        s_wheel = spread(self.rel_wheel)
        s_vins = spread(self.rel_vins)
        sep_med = float(np.median(self.seps))
        # 판별력의 핵심은 벌어짐의 "크기"가 아니라 표본 구간 동안의 "변화량"이다.
        # 로봇이 정지해 있으면 두 자가 함께 멈춰서 (벌어짐이 163mm라도) 양쪽 산포가
        # 전부 노이즈 수준(<1mm)으로 나와 무의미한 판정이 된다 — 08-20 실기에서 실증.
        sep_range = float(np.max(self.seps) - np.min(self.seps))
        print('\n===== 앵커 자 판별 결과 =====')
        print(f'표본 수            : {len(self.rel_wheel)}')
        print(f'두 자 벌어짐(중앙)  : {sep_med*1000:.1f} mm')
        print(f'벌어짐 변화량       : {sep_range*1000:.1f} mm  (판별 하한 {SEP_MIN_M*1000:.0f} mm)')
        print(f'wheel 자 산포      : {s_wheel*1000:.2f} mm')
        print(f'VINS  자 산포      : {s_vins*1000:.2f} mm')
        if sep_range < SEP_MIN_M:
            print('⚠️ 판별 불가 — 표본 구간 동안 두 자의 상대 변화가 없음(로봇 정지?).')
            print('   마커가 보이는 상태에서 로봇을 움직이거나 제자리회전하며 재실행할 것.')
            return
        # 승부가 노이즈 수준이면 단정하지 않는다
        if max(s_wheel, s_vins) < 2.0 * min(s_wheel, s_vins):
            print('⚠️ 판별 불가 — 양쪽 산포 차이가 2배 미만(노이즈 우열). 더 크게 움직이며 재실행.')
            return
        if s_vins < s_wheel:
            print('✅ 앵커는 VINS 자(base_vins) — robot_pose_frame 선택자 정상 작동')
        else:
            print('❌ 앵커는 wheel 자(base_footprint) — 선택자 미적용/미배포 의심 '
                  '(yaml robot_pose_frame=base_vins? 3d96e6e 배포? HF publish_hf_body_tf=1?)')

    def report_z(self, anchor, vins_z):
        """z 기반 결정 판별 (08-20 확립 — 통계 방식보다 우선 신뢰).

        원리: 판별에는 참값 기준이 필요한데 xy·yaw는 마커의 odom 참위치를 몰라 불가.
        z만 '바닥'이라는 절대 기준이 있다 — 휠 자는 z≡0(바닥 고정)이므로
          휠 자 가설  : 실제 마커 높이 = anchor_z
          VINS 자 가설: 실제 마커 높이 = anchor_z − vins_z
        둘 중 줄자 실측 높이에 가까운 쪽이 정답. (08-20 실기: 실측 18.5cm →
        휠 8.5cm vs VINS 20.1cm, VINS 자 확정)
        """
        az = anchor.transform.translation.z
        print('\n===== z 기반 결정 판별 (줄자로 마커 중심 높이 실측 후 대조) =====')
        print(f'휠 자 가설  → 실제 마커 높이 = {az:+.4f} m')
        print(f'VINS 자 가설 → 실제 마커 높이 = {az - vins_z:+.4f} m')
        print('실측 높이에 가까운 쪽이 앵커에 실사용된 자.')


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    rclpy.init()
    node = RulerCheck(n)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
