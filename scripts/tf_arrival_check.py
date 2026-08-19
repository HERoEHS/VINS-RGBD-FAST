#!/usr/bin/env python3
"""/tf 에서 프레임별 도착 지연·발행 주기 실측 (로봇 PC에서 실행 — 같은 시계여야 지연이 순수함).

  도착 지연 = 수신 시각(now) − header.stamp
  station_docking 이 lookup(t_img, timeout 0.1s) 로 성공하려면
  base_vins 샘플의 (발행 간격 + 도착 지연) ≤ 검출 소요 + 0.1s 여야 한다.

사용:  python3 tf_arrival_check.py [초] [프레임,프레임,...]
기본:  10초, base_vins,body,base_footprint
"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage


class ArrivalCheck(Node):
    def __init__(self, frames):
        super().__init__("tf_arrival_check")
        self.frames = frames
        self.delay = {f: [] for f in frames}       # now - stamp
        self.stamps = {f: [] for f in frames}      # header.stamp (발행 간격용)
        self.bundled_with_body = 0                 # base_vins가 body와 같은 메시지
        self.base_vins_msgs = 0
        # BEST_EFFORT: 무선 경로에서 재전송 큐잉(08-05 현상)으로 측정 자체가 지연을 만들지 않게.
        # 발행자(tf2_ros)가 RELIABLE이어도 호환된다. 로봇 PC 로컬에서는 어느 쪽이든 무관.
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(TFMessage, "/tf", self.cb, qos)

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        childs = [t.child_frame_id for t in msg.transforms]
        if "base_vins" in childs:
            self.base_vins_msgs += 1
            if "body" in childs:
                self.bundled_with_body += 1
        for t in msg.transforms:
            f = t.child_frame_id
            if f in self.delay:
                st = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                self.delay[f].append(now - st)
                self.stamps[f].append(st)


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    frames = sys.argv[2].split(",") if len(sys.argv) > 2 else ["base_vins", "body", "base_footprint"]
    rclpy.init()
    n = ArrivalCheck(frames)
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.05)

    print(f"\n[{dur:.0f}s 수집]  프레임별 도착 지연(now−stamp)과 발행 주기")
    print(f"{'frame':16} {'n':>5} {'rate[Hz]':>9} {'delay med':>10} {'p95':>8} {'max':>8}   판정")
    for f in frames:
        d = np.array(n.delay[f]); s = np.array(n.stamps[f])
        if len(d) < 2:
            print(f"{f:16} {len(d):>5}   (샘플 없음 — 발행 안 되거나 프레임명 다름)"); continue
        rate = 1.0 / np.median(np.diff(np.sort(s)))
        med, p95, mx = np.median(d) * 1e3, np.percentile(d, 95) * 1e3, d.max() * 1e3
        if f == "base_vins":
            # 20Hz면 t_img 이후 첫 샘플까지 ≤50ms, 그 샘플 도착까지 +delay. 검출 ~100ms+0.1s 여유 → 150ms 이내면 안전
            budget_ms = 1000.0 / rate + p95
            verdict = "✅ HF 경로 (lookup 여유 충분)" if p95 < 60 else ("⚠️ 저주기 경로로 보임 — publish_hf_body_tf 확인" if p95 > 80 else "△ 경계")
            verdict += f"  [간격+p95 ≈ {budget_ms:.0f}ms]"
        elif f == "body":
            verdict = "VINS map→body (base_vins와 같아야 함)"
        elif f == "base_footprint":
            verdict = "diff_drive 기준선 (~0)"
        else:
            verdict = ""
        print(f"{f:16} {len(d):>5} {rate:>9.1f} {med:>9.1f}ms {p95:>7.1f}ms {mx:>7.1f}ms   {verdict}")
    if n.base_vins_msgs:
        print(f"\n[묶음 발행] base_vins 메시지 {n.base_vins_msgs}개 중 body와 같은 메시지: {n.bundled_with_body}개"
              f"  → {'✅ 패킷 수 불변' if n.bundled_with_body == n.base_vins_msgs else '⚠️ 분리 발행 있음'}")
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
