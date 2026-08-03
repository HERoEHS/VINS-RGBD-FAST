#pragma once
// [SW1-1866] init/출발 워밍업 게이트 — 판정 로직 (헤더 단독, gtest 가능)
//
// [왜] (doc/WARMUP_GATE_PROPOSAL.md 실증 3건)
//   ⓐ 재초기화가 다리 애니메이션 중 착지(07-31 실기): init acc 평균 (1.06, 1.75, 9.43)
//     = 흔들리는 몸의 중력 평균 → R0에 ~10° tilt를 구워 넣음 → 비강체 절제·슬라이드 연쇄.
//   ⓑ 부팅 무휴식 시작(08-03 v13): Bg 미수렴 상태로 init → 초기 yaw +3~4.5° 고착
//     → 병진 시 횡(y) 오차로 전사.
//   처방: static init 창 수집을 "몸이 실제로 정지·정착했음이 실증되고 충분한 관측이
//   쌓인 뒤"로 지연한다.
//
// [왜 상태 실증 + 표본 수인가 — 시간 상수 금지]
//   "부팅 후 N초"류는 검증 bag 안무 의존 상수라 금지(bgz_lock과 동일 원칙).
//   선례 = ALICE 상태추정기 init: 양발 접지 '연속 유지' 실증(상태) → IMU 100샘플
//   수집(관측량) → init. 본 게이트도 동일 2단 구조:
//     1단: 휠 정지 AND 다리각 안정이 '연속 표본 수'로 실증될 때까지 대기
//     2단: 실증 유지 상태에서 IMU 표본 N개 축적 → READY
//   도중 움직임이 관측되면 두 카운터 모두 리셋(오염된 창 폐기).
//
// [왜 폴백이 하드 요건인가 — R2]
//   운용 중 재초기화(에스컬레이션 reboot) 순간 로봇이 계속 주행 명령을 받고 있으면
//   1단이 영원히 미충족 → VINS 출력 공백 무한 연장. 폴백 2중 장치:
//     ① 조기 폴백: '주행 중' 연속 표본 실증(움직임 자체의 상태 실증) → 즉시 저신뢰 init
//     ② 예산 폴백: 총 관측 표본이 예산을 소진하도록 READY 미달(간헐 움직임 등
//        ①이 못 잡는 혼합 패턴) → 저신뢰 init. 무한 대기 금지의 최종 보장.
//   폴백 후에도 관측은 계속한다 — 이후 정지가 도래해 1·2단이 충족되면 READY로
//   전이해 "재정렬 기회" 신호를 준다(fellBack()으로 잠정 init 여부 구분).
//
// [입력 계약 — R4: 기존 판정 기계 재사용]
//   wheel_still / leg_stable은 호출측(estimator)의 기존 정지 3중 검사
//   (bgz_lock::kStillWheelMax 휠 문턱 + 다리 이벤트 게이트)가 계산한 값을 그대로
//   받는다. 새 판정기를 만들지 않는다. 호출은 IMU 표본 1개당 1회(=표본 카운트의
//   단위가 IMU 샘플로 통일됨).
//
// 순수 로직(ROS/Eigen 미의존) — 스레드 안전 아님(호출측 mutex).
namespace warmup_init_gate
{

struct Params
{
    // 1단: 정지 실증 연속 표본 수. IMU ~100Hz 기준 100 ≈ ALICE '접지 1s 연속' 선례.
    //   <=0 이면 게이트 전체 비활성(항상 READY = 기존 동작, 안전 기본).
    int still_samples   = 0;
    // 2단: 실증 유지 중 축적할 IMU 표본 수 (ALICE 100샘플 선례)
    int imu_samples     = 100;
    // 조기 폴백: '휠 주행 중' 연속 표본 수 — 이만큼 연속으로 바퀴가 돌면
    //   "로봇은 지금 확실히 주행 중"의 상태 실증으로 보고 즉시 저신뢰 init.
    //   <=0 이면 조기 폴백 없음(예산 폴백만).
    int moving_samples  = 100;
    // 예산 폴백: 게이트 시작 후 총 관측 표본 상한 — 소진 시 READY 미달이면
    //   저신뢰 init(무한 대기 금지 하드 보장). <=0 이면 예산 무한(권장 안 함 —
    //   조기 폴백마저 끄면 무한 대기가 가능해지므로 배선 시 양수 필수).
    int budget_samples  = 3000;
};

enum class Verdict
{
    WAIT,      // 실증 진행 중 — init 보류
    READY,     // 정지 실증 + 표본 축적 완료 — 정상 init (폴백 후라면 재정렬 기회)
    FALLBACK,  // 주행 실증 or 예산 소진 — 저신뢰 init(가드 강화) 진행
};

class Gate
{
public:
    void setParams(const Params &p) { p_ = p; }

    // IMU 표본 1개당 1회 호출. wheel_still/leg_stable = 호출측 기존 판정(R4).
    void onSample(bool wheel_still, bool leg_stable)
    {
        if (p_.still_samples <= 0)
            return;  // 비활성 — verdict()가 항상 READY
        if (ready_)
            return;  // READY는 종착 상태(정상 init 또는 재정렬 신호 후 소비 완료)

        ++total_;

        // 1·2단 카운터 — 움직임 관측 시 오염 창 폐기(둘 다 리셋)
        if (wheel_still && leg_stable)
        {
            ++still_streak_;
            if (still_streak_ > p_.still_samples)
                ++imu_collected_;  // 상태 실증 완료 후에만 관측 축적(ALICE 2단)
        }
        else
        {
            still_streak_ = 0;
            imu_collected_ = 0;
        }

        // 조기 폴백용 '주행 실증' — 휠만 본다(다리 애니메이션 단독은 주행이 아님;
        // 다리만 계속 움직이는 유휴 애니메이션 케이스는 예산 폴백이 커버)
        moving_streak_ = wheel_still ? 0 : moving_streak_ + 1;

        if (imu_collected_ >= p_.imu_samples)
        {
            ready_ = true;  // 폴백 이후였다면 "재정렬 기회" 신호로 승격
            return;
        }
        if (!fell_back_)
        {
            const bool early  = p_.moving_samples > 0 && moving_streak_ >= p_.moving_samples;
            const bool budget = p_.budget_samples > 0 && total_ >= p_.budget_samples;
            if (early || budget)
                fell_back_ = true;
        }
    }

    Verdict verdict() const
    {
        if (p_.still_samples <= 0)
            return Verdict::READY;  // 비활성 = 기존 동작
        if (ready_)
            return Verdict::READY;
        return fell_back_ ? Verdict::FALLBACK : Verdict::WAIT;
    }

    // 저신뢰(잠정) init 경로를 탔는가 — READY 전이 후에도 유지(가드 강화·재정렬 판단용)
    bool fellBack() const { return fell_back_; }

    // 재초기화 경로(clearState)에서 반드시 호출 — GUARD_RESET_PATH_CHECKLIST Q3:
    // 병리 중 리셋이 게이트를 우회하지 못하도록 전 상태 초기화(파라미터는 유지)
    void reset()
    {
        still_streak_ = imu_collected_ = moving_streak_ = total_ = 0;
        ready_ = fell_back_ = false;
    }

    // 진단 로그용
    int stillStreak() const { return still_streak_; }
    int imuCollected() const { return imu_collected_; }
    int totalSamples() const { return total_; }

private:
    Params p_;
    int  still_streak_ = 0;   // 1단: 정지(휠+다리) 연속 실증 표본 수
    int  imu_collected_ = 0;  // 2단: 실증 유지 중 축적된 IMU 표본 수
    int  moving_streak_ = 0;  // 조기 폴백: 휠 주행 연속 실증 표본 수
    int  total_ = 0;          // 예산 폴백: 게이트 시작 후 총 관측 표본 수
    bool ready_ = false;
    bool fell_back_ = false;
};

}  // namespace warmup_init_gate
