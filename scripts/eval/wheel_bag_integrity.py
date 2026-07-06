#!/usr/bin/env python3
# wheel odom bag 무결성 검증 — 재녹화한 _gt가 깨끗한지 재실행 전에 확인.
#   체크: ① header stamp 중복/단조성/dt 갭(>0.5s)  ② 물리한계(|v|>0.6, |w|>3.3) 초과 샘플
#         ③ 궤적 기하(bounding box·PCA·닫힌루프 끝점)
# 사용: python3 wheel_bag_integrity.py <bag경로> [--topic /edie/diff_drive_controller/odom]
import os
import sys
import yaml
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# 로봇 물리 한계 — vio_edie.yaml wheel_vel_max/wheel_gyr_max 와 동일하게 맞춤.
#   실측 포화: |v|=0.5 m/s, |w|=3.0 rad/s. 여유 추가해 0.6 / 3.3 (= VINS factor-skip 게이트와 동일).
#   ⇒ 이 스크립트가 ❌로 잡는 샘플은 VINS use_wheel_vel_gate가 실제로 skip 하는 샘플과 일치.
V_PHYS, W_PHYS = 0.6, 3.3
GAP_THRESH = 0.5  # header dt 갭 경고 임계 [s]


def detect_storage_id(bag_path):
    """bag 폴더의 metadata.yaml에서 storage_identifier 자동 감지 (sqlite3 vs mcap 등)."""
    meta_path = os.path.join(bag_path, "metadata.yaml")
    if os.path.isfile(meta_path):
        try:
            with open(meta_path) as f:
                meta = yaml.safe_load(f)
            return meta["rosbag2_bagfile_information"]["storage_identifier"]
        except (KeyError, yaml.YAMLError):
            pass
    return "sqlite3"  # fallback

def main():
    bag = sys.argv[1]
    topic = "/edie/diff_drive_controller/odom"
    if "--topic" in sys.argv:
        topic = sys.argv[sys.argv.index("--topic") + 1]

    storage_id = detect_storage_id(bag)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage_id),
                rosbag2_py.ConverterOptions("", ""))

    # 재생 명령 결정용 — bag에 /clock 토픽이 이미 녹화돼 있는지 확인.
    #   YES: ros2 bag play <bag>   (--clock 옵션 불필요, 중복 publish 충돌 위험)
    #   NO : ros2 bag play <bag> --clock   (use_sim_time:=true 노드들이 따라가게 publish 필요)
    topic_names = [t.name for t in reader.get_all_topics_and_types()]
    has_clock = "/clock" in topic_names
    print(f"[bag info] storage={storage_id}, /clock 토픽: "
          f"{'YES (재생 시 --clock 불필요)' if has_clock else 'NO (재생 시 --clock 권장 + use_sim_time:=true)'}")

    mt = get_message("nav_msgs/msg/Odometry")
    ts, lin, ang, pos = [], [], [], []
    while reader.has_next():
        tp, data, _ = reader.read_next()
        if tp != topic:
            continue
        m = deserialize_message(data, mt)
        ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        v, w, p = m.twist.twist.linear, m.twist.twist.angular, m.pose.pose.position
        lin.append(np.sqrt(v.x**2 + v.y**2 + v.z**2))
        ang.append(np.sqrt(w.x**2 + w.y**2 + w.z**2))
        pos.append([p.x, p.y, p.z])
    if not ts:
        print(f"❌ '{topic}' 메시지 0개 — 토픽명 확인 필요")
        return
    ts = np.array(ts); lin = np.array(lin); ang = np.array(ang); pos = np.array(pos)
    dts = np.diff(ts)
    issues = []

    print(f"===== {bag.rstrip('/').split('/')[-1]} =====")
    print(f"메시지 {len(ts)}개, 구간 {ts[-1]-ts[0]:.1f}s")

    # ① header stamp 무결성
    dup = len(ts) - len(np.unique(ts))
    nonmono = int((dts <= 0).sum())
    gaps = np.where(dts > GAP_THRESH)[0]
    print(f"\n[① 타임스탬프]")
    print(f"  중복 stamp        : {dup} {'❌' if dup else '✓'}")
    print(f"  비단조(dt<=0)     : {nonmono} {'❌' if nonmono else '✓'}")
    print(f"  dt median/max     : {np.median(dts):.5f}s / {dts.max():.5f}s")
    print(f"  큰 갭(>{GAP_THRESH}s)      : {len(gaps)} {'❌' if len(gaps) else '✓'}")
    for g in gaps[:5]:
        print(f"     t={ts[g]-ts[0]:.3f}s 에서 dt={dts[g]:.3f}s 갭")
    if dup: issues.append(f"중복 stamp {dup}")
    if nonmono: issues.append(f"비단조 {nonmono}")
    if len(gaps): issues.append(f"dt갭 {len(gaps)}")

    # ② 물리한계 초과 (비물리 글리치)
    v_bad = int((lin > V_PHYS).sum()); w_bad = int((ang > W_PHYS).sum())
    print(f"\n[② 물리한계 초과] (|v|>{V_PHYS} m/s, |w|>{W_PHYS} rad/s)")
    print(f"  |v| max/p99.9     : {lin.max():.3f} / {np.percentile(lin,99.9):.3f} m/s")
    print(f"  |w| max/p99.9     : {ang.max():.3f} / {np.percentile(ang,99.9):.3f} rad/s")
    print(f"  |v| 초과 샘플     : {v_bad} ({100*v_bad/len(lin):.3f}%) {'❌' if v_bad else '✓'}")
    print(f"  |w| 초과 샘플     : {w_bad} ({100*w_bad/len(ang):.3f}%) {'❌' if w_bad else '✓'}")
    if v_bad: issues.append(f"|v|초과 {v_bad}")
    if w_bad: issues.append(f"|w|초과 {w_bad}")

    # ③ 궤적 기하 (사각형 루프·닫힌루프 끝점)
    pos -= pos[0]
    endpoint = np.linalg.norm(pos[-1])
    xy = pos[:, :2]
    # PCA 주축 길이(사각형 가로/세로 가늠)
    u, s, vt = np.linalg.svd(xy - xy.mean(0), full_matrices=False)
    proj = (xy - xy.mean(0)) @ vt.T
    dim1, dim2 = proj[:,0].ptp(), proj[:,1].ptp()
    print(f"\n[③ 궤적 기하]")
    print(f"  총 이동거리       : {np.linalg.norm(np.diff(pos,axis=0),axis=1).sum():.2f} m")
    print(f"  PCA 주축(가로×세로): {dim1:.2f} × {dim2:.2f} m")
    print(f"  닫힌루프 끝점     : {endpoint:.3f} m (시작점 복귀 가정)")

    # 판정 = "오염 검출"(신뢰) + "발산위험 가설"(참고). bag 통계만으론 발산 예측 불가
    #   (base_old: max|w|=292인데 use_wheel:1 견딤 → 각속도 max는 발산 예측 안 됨).
    #   관측(n=3): 발산을 가른 건 선속도 |v| (_gt 106=발산 / base 26·base_old 11=견딤).
    #   ★실제 발산 여부는 VINS 실행만이 판정. 아래 위험도는 참고용 가설.
    vmax, wmax = lin.max(), ang.max()
    if not issues:
        print(f"\n[판정] ✅ CLEAN — 글리치 없음. use_wheel:1 그대로 신뢰 OK")
    else:
        print(f"\n[판정] ⚠️ 오염 검출 — 세부: {', '.join(issues)}")
        risk = "높음(_gt수준, |v|발산 사례)" if vmax > 50 else "낮음~중(base수준, 과거 use_wheel:1 견딤)"
        print(f"       발산위험(가설, |v|기준): {risk}  | max|v|={vmax:.0f} m/s, max|w|={wmax:.0f} rad/s")
        print(f"       ※ bag 통계는 발산을 예측 못 함 — 실제 판정은 VINS 실행 A/B 필수")

if __name__ == "__main__":
    main()
