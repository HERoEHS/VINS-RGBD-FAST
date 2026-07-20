#!/usr/bin/env python3
"""[SW1-1837] edie_calib_verify bag 검수 — ROBOT_TASK_v6_record.md 스펙 자동 판정.

사용:
    python3 bag_acceptance_check.py <bag_경로> [--yaml imu_offsets_sN.yaml]

판정 항목 (각각 PASS/WARN/FAIL + 실측값):
    C1  필수 토픽 존재·수신량
    C2  길이 90~180s
    C3  IMU 건강 (레이트 ~375Hz, 최대 갭)
    C4  시작 정지 5s+ (VINS static init)
    C5  측정용 정지(2.5s+) 개수
    C6  정지 heading 4방위 커버 (+시간-heading 뒤엉킴 참고 지표)
    C7  초반(1/3) 원거리(90~270°) heading 정지  — 뒤엉킴 해소 샘플 ①
    C8  말기(2/3~) 초기 heading(±30°) 정지      — 뒤엉킴 해소 샘플 ②
    C9  다리 미사용 (정지 중 '움직임/기준 자세 이탈'만 문제 — 주차 각도는 무해)
    C10 twist 검진 (적분비 2종 ≈ 1.0 — 휠 오염 판별, 기존 표준)
    C11 (--yaml 시) 캘리브 품질 키·레벨링 각
    C12 시작·종료 정지 중 카메라 프레임 존재 (AprilTag GT 확보 대리 지표)
"""
import argparse
import math
import sys

import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry

IMU_TOPIC = '/edie/sensor/offset_imu'
ODOM_TOPIC = '/edie/diff_drive_controller/odom'
JOINT_TOPIC = '/joint_states'
CAM_TOPIC = '/edie/sensors/camera/left/image_gray'
REQUIRED = [IMU_TOPIC, '/edie/sensor/lpf_imu', ODOM_TOPIC, JOINT_TOPIC,
            CAM_TOPIC, '/edie/sensors/camera/left/depth', '/tf', '/tf_static']

results = []


def report(code, status, msg):
    results.append(status)
    icon = {'PASS': '✅', 'WARN': '⚠️ ', 'FAIL': '❌'}[status]
    print(f"{icon} [{code}] {status}: {msg}")


def yaw_of(q):
    # nav_msgs orientation → yaw
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ang_norm(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--yaml', default=None)
    args = ap.parse_args()

    # ---- 단일 패스 읽기: 내용은 IMU/odom/joints만 역직렬화, 나머지는 개수만 ----
    r = SequentialReader()
    r.open(StorageOptions(uri=args.bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    counts = {}
    imu_t, acc, gyr = [], [], []
    od_t, od_v, od_w, od_xy, od_yaw = [], [], [], [], []
    jt, jleg = [], []
    leg_idx = None
    while r.has_next():
        topic, data, _ = r.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if topic == IMU_TOPIC:
            m = deserialize_message(data, Imu)
            imu_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            acc.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
            gyr.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        elif topic == ODOM_TOPIC:
            m = deserialize_message(data, Odometry)
            od_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            od_v.append(m.twist.twist.linear.x)
            od_w.append(m.twist.twist.angular.z)
            od_xy.append([m.pose.pose.position.x, m.pose.pose.position.y])
            od_yaw.append(yaw_of(m.pose.pose.orientation))
        elif topic == JOINT_TOPIC:
            m = deserialize_message(data, JointState)
            if leg_idx is None:
                leg_idx = [i for i, n in enumerate(m.name) if 'leg' in n.lower()]
            jt.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            jleg.append(max(abs(m.position[i]) for i in leg_idx) if leg_idx else 0.0)

    imu_t = np.array(imu_t); acc = np.array(acc); gyr = np.array(gyr)
    od_t = np.array(od_t); od_v = np.array(od_v); od_w = np.array(od_w)
    od_xy = np.array(od_xy); od_yaw = np.unwrap(np.array(od_yaw))
    jt = np.array(jt); jleg = np.array(jleg)
    if len(imu_t) == 0:
        print("❌ IMU 데이터 없음 — 검수 불가"); sys.exit(2)
    t0, t1 = imu_t[0], imu_t[-1]
    dur = t1 - t0
    print(f"bag: {args.bag}  길이 {dur:.1f}s")

    # C1 토픽
    missing = [t for t in REQUIRED if counts.get(t, 0) == 0]
    if missing:
        report('C1', 'FAIL', f"누락 토픽: {missing}")
    else:
        report('C1', 'PASS', f"필수 토픽 8종 수신 (imu {counts[IMU_TOPIC]}, odom {counts[ODOM_TOPIC]}, "
                             f"joint {counts[JOINT_TOPIC]}, cam {counts.get(CAM_TOPIC, 0)})")

    # C2 길이
    st = 'PASS' if 90 <= dur <= 180 else ('WARN' if 60 <= dur <= 240 else 'FAIL')
    report('C2', st, f"길이 {dur:.1f}s (목표 90~180)")

    # C3 IMU 건강
    rate = len(imu_t) / dur
    gap = float(np.max(np.diff(imu_t))) * 1000
    st = 'PASS' if abs(rate - 375) / 375 < 0.10 and gap < 20 else ('WARN' if gap < 100 else 'FAIL')
    report('C3', st, f"IMU {rate:.0f}Hz, 최대 갭 {gap:.1f}ms (목표 ~375Hz, 갭<20ms)")

    # ---- 정지 검출 (IMU 정온 AND 휠 정지 — IMU만 쓰면 저속 직진을 정지로 오인해 병합) ----
    gn = np.linalg.norm(gyr, axis=1)
    win_t, win_ok = [], []
    for e in np.arange(t0, t1, 0.2):
        msk = (imu_t >= e) & (imu_t < e + 0.6)
        if msk.sum() < 15:
            continue
        mo = (od_t >= e) & (od_t < e + 0.6)
        wheel_still = mo.any() and np.abs(od_v[mo]).max() < 0.02 and np.abs(od_w[mo]).max() < 0.05
        win_t.append(e + 0.3)
        win_ok.append(wheel_still and gn[msk].max() < 0.05 and acc[msk].std(axis=0).max() < 0.15)
    stops = []
    cur = None
    for tt_, ok in zip(win_t, win_ok):
        if ok:
            cur = [tt_, tt_] if cur is None else [cur[0], tt_]
        elif cur is not None:
            stops.append(cur); cur = None
    if cur is not None:
        stops.append(cur)
    stops = [(a - 0.3, b + 0.3) for a, b in stops]

    # 다리 판정 — 오염원은 다리 '움직임'과 '세션 기준 자세 이탈'이지 절대각이 아니다.
    # (다리각은 1° 양자화 — 2° 주차 자세를 절대각 문턱으로 자르면 오탐. v6 s1 실측 교훈)
    leg_base = math.degrees(float(np.median(jleg))) if len(jleg) else 0.0

    def leg_motion(a, b):
        """정지 창 '내부'의 다리 움직임 폭[deg]과 기준 자세 이탈[deg].
        여유(margin)를 두지 않는다 — 정지 그룹은 IMU 정온이 보장된 구간이라 다리 스윙이
        구간 안에 있을 수 없고, 여유를 주면 경계 직후의 스윙이 긴 정지를 통째로 무효화한다."""
        m = (jt >= a) & (jt <= b)
        if not m.any():
            return 0.0, 0.0
        d = np.degrees(jleg[m])
        return float(d.max() - d.min()), float(abs(np.mean(d) - leg_base))

    def leg_ok(a, b):
        mot, dev = leg_motion(a, b)
        return mot < 1.5 and dev < 2.0  # 움직임 <1.5°(양자화 1~2 LSB 허용) AND 기준 이탈 <2°

    meas = [(a, b) for a, b in stops if (b - a) >= 2.5 and leg_ok(a, b)]

    # C4 시작 정지
    start_ok = stops and stops[0][0] - t0 < 1.0 and (stops[0][1] - t0) >= 5.0
    report('C4', 'PASS' if start_ok else 'FAIL',
           f"시작 정지 {stops[0][1] - t0:.1f}s (목표 5s+)" if stops else "시작 정지 없음")

    # C5 측정용 정지 수
    st = 'PASS' if len(meas) >= 8 else ('WARN' if len(meas) >= 5 else 'FAIL')
    report('C5', st, f"측정용 정지(2.5s+·다리 정지) {len(meas)}개 (목표 8+): "
                     + ", ".join(f"{(a + b) / 2 - t0:.0f}s" for a, b in meas))

    # 정지별 heading (휠 odom yaw — 빈 분류엔 스케일 오차 무관)
    def stop_yaw(a, b):
        m = (od_t >= a) & (od_t <= b)
        return float(np.mean(od_yaw[m])) if m.any() else None

    yaw0 = stop_yaw(*meas[0]) if meas else None
    rels = []
    for a, b in meas:
        y = stop_yaw(a, b)
        if y is not None and yaw0 is not None:
            rels.append((((a + b) / 2 - t0), math.degrees(ang_norm(y - yaw0))))

    # C6 4방위 커버 + 시간-heading 뒤엉킴 참고 지표
    bins = set()
    for _, d in rels:
        bins.add(int(((d + 45) % 360) // 90))
    st = 'PASS' if len(bins) >= 4 else ('WARN' if len(bins) == 3 else 'FAIL')
    confound = ''
    if len(rels) >= 4:
        tt_ = np.array([r_[0] for r_ in rels]); dd_ = np.abs([r_[1] for r_ in rels])
        c = float(np.corrcoef(tt_, dd_)[0, 1])
        confound = f" | corr(시간,|heading|)={c:.2f} (|c|<0.5면 뒤엉킴 낮음 — C7/C8의 실질 지표)"
    report('C6', st, f"heading 방위 커버 {len(bins)}/4 (정지 heading: "
                     + ", ".join(f"{d:.0f}°@{tc:.0f}s" for tc, d in rels) + ")" + confound)

    # C7 초반 원거리 정지
    ok = any(tc < dur / 3 and 90 <= abs(d) <= 270 for tc, d in rels)
    report('C7', 'PASS' if ok else 'FAIL', "초반(1/3) 원거리(90~270°) 정지 " + ("있음" if ok else "없음 — 샘플 ①"))

    # C8 말기 초기 heading 정지
    ok = any(tc > 2 * dur / 3 and abs(d) < 30 for tc, d in rels)
    report('C8', 'PASS' if ok else 'FAIL', "말기(2/3~) 초기 heading(±30°) 정지 " + ("있음" if ok else "없음 — 샘플 ②"))

    # C9 다리 — 정지 중 '움직임/기준 이탈'만 문제 삼는다 (주차 자세는 무해)
    leg_all = math.degrees(jleg.max()) if len(jleg) else 0.0
    dirty = []
    for a, b in stops:
        if (b - a) >= 2.5 and not leg_ok(a, b):
            mot, dev = leg_motion(a, b)
            dirty.append(f"{(a + b) / 2 - t0:.0f}s(움직임 {mot:.1f}°/이탈 {dev:.1f}°)")
    if dirty:
        report('C9', 'FAIL', f"정지 중 다리 이벤트: {dirty} — 해당 정지 측정 무효 (기준자세 {leg_base:.1f}°)")
    elif leg_all - leg_base > 2.0:
        report('C9', 'WARN', f"다리 사용 감지(최대 {leg_all:.1f}°, 기준 {leg_base:.1f}°)이나 정지와 미중첩 — 측정엔 무해")
    else:
        report('C9', 'PASS', f"다리 미사용 (최대 {leg_all:.1f}°, 기준자세 {leg_base:.1f}°)")

    # C10 twist 검진 (기존 표준: 적분비 2종)
    dt_od = np.diff(od_t)
    yaw_tw = float(np.sum(np.abs(od_w[:-1]) * dt_od))
    dt_imu = np.diff(imu_t)
    yaw_imu = float(np.sum(np.abs(gyr[:-1, 2]) * dt_imu))
    r1 = yaw_tw / yaw_imu if yaw_imu > 1e-6 else float('nan')
    dist_tw = float(np.sum(np.abs(od_v[:-1]) * dt_od))
    path = float(np.sum(np.linalg.norm(np.diff(od_xy, axis=0), axis=1)))
    r2 = dist_tw / path if path > 1e-6 else float('nan')
    ok1, ok2 = 0.9 <= r1 <= 1.1, 0.9 <= r2 <= 1.1
    st = 'PASS' if ok1 and ok2 else ('WARN' if 0.8 <= r1 <= 1.2 and 0.8 <= r2 <= 1.2 else 'FAIL')
    report('C10', st, f"twist/IMU yaw 적분비 {r1:.3f}, ∫|v|/경로 {r2:.3f} (목표 각 0.9~1.1)")

    # C11 yaml (선택)
    if args.yaml:
        try:
            import yaml as _yaml
            cfg = _yaml.safe_load(open(args.yaml))
            if 'level_angle_deg' in cfg:
                lvl = float(cfg['level_angle_deg'])
                st = 'PASS' if lvl <= 5.0 else 'WARN'
                report('C11', st, f"레벨링 각 {lvl:.2f}° (calibrated_at: {cfg.get('calibrated_at', '없음')})")
            else:
                report('C11', 'FAIL', "level_angle_deg 키 없음 — 구버전 wrapper로 캘리브된 세션")
        except Exception as e:  # yaml 파일 문제는 검수 실패로 취급
            report('C11', 'FAIL', f"yaml 읽기 실패: {e}")

    # C12 카메라 (AprilTag GT 대리 지표 — 존재·양만, 실제 태그 검출은 GT 파이프라인에서)
    cam_n = counts.get(CAM_TOPIC, 0)
    st = 'PASS' if cam_n / dur > 5 else ('WARN' if cam_n > 0 else 'FAIL')
    report('C12', st, f"카메라 {cam_n}프레임({cam_n / dur:.0f}Hz) — 시작·종료 태그 시야는 GT 추출로 최종 확인")

    # ---- 종합 ----
    n_fail = results.count('FAIL')
    n_warn = results.count('WARN')
    print("\n" + "=" * 60)
    if n_fail == 0 and n_warn == 0:
        print("🟢 합격 — 모든 조건 충족")
    elif n_fail == 0:
        print(f"🟡 조건부 합격 — WARN {n_warn}건 확인 필요")
    else:
        print(f"🔴 불합격 — FAIL {n_fail}건 (재녹화 권장), WARN {n_warn}건")
    sys.exit(1 if n_fail else 0)


if __name__ == '__main__':
    main()
