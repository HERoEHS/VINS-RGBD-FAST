#pragma once
// [SW1-1889] depth 채택 프로브 — 최적화마다 "VINS가 depth 이미지를 고정 채택한 특징점"을 파일로 남긴다.
//
// 왜 필요한가: depth 백엔드 A/B(SGBM vs OpenCL BM)에서 BM은 신선 유효 depth가 5~8 %뿐인데도
// VINS 닫힌루프 오차가 SGBM과 같았다. 발행 depth는 EMA(α0.4)가 옛값을 유지하므로,
// VINS가 실제로 몇 점에 depth를 고정했는지(estimate_flag==1 && FIX_DEPTH)와 그 값이
// 몇 프레임 묵었는지를 알아야 판정할 수 있다. 나이는 오프라인 age 맵(스탬프·화소)과
// 관측 행(O)을 결합해 구한다. 환경변수 VINS_DEPTH_ADOPT_LOG=<파일경로> 로만 켜진다(기본 off).
//
// 행 형식(공백 구분, 한 줄 한 레코드):
//   S <t> <fixed> <tri> <rough> <total> <flag1>            최적화 1회 요약: 잔차 참여 특징 중 depth 고정(flag 1 && FIX_DEPTH) /
//                                                          삼각측량(flag 2) / 그 외 수, flag1 = FIX_DEPTH 와 무관한 flag 1 수
//   A <t> <id> <depth_m>                                    처음 고정 채택된 특징(한 번만). depth_m 은 앵커 프레임으로 옮긴 verified 평균
//   O <t> <id> <frame_stamp> <u> <v> <depth_m> <verified>   그 특징의 관측 중 0 < depth ≤ max_dist 인 것. verified = 재투영
//                                                          교차검증 통과 횟수(0 = 후보였을 뿐). 채택값은 이 횟수로 가중된 평균이므로
//                                                          나이도 이 값을 가중치로 집계한다
#include <cstdio>
#include <fstream>
#include <set>
#include <string>

namespace depth_adopt_probe
{
struct Counts
{
    int fixed = 0;  // estimate_flag 1 && FIX_DEPTH → 파라미터 블록 상수 고정
    int tri   = 0;  // estimate_flag 2 → 삼각측량(상한만)
    int rough = 0;  // 그 외(초기값·평균, FIX_DEPTH=0 인 flag 1 포함)
    int total = 0;
    int flag1 = 0;  // estimate_flag 1 (FIX_DEPTH 무관) — fix_depth 설정과 분리해 세는 히스토그램
};

// 잔차에 참여한 특징 하나를 분류에 더한다.
inline void count(int estimate_flag, bool fix_depth, Counts &c)
{
    c.total++;
    if (estimate_flag == 1)
        c.flag1++;
    if (estimate_flag == 1 && fix_depth)
        c.fixed++;
    else if (estimate_flag == 2)
        c.tri++;
    else
        c.rough++;
}

// depth 이미지 관측이 "검증 depth" 후보인가(triangulateWithDepth 의 verified 범위와 동일; 교차검증은 verified 플래그가 담당).
inline bool obsHasDepth(double depth_m, double max_dist)
{
    return depth_m > 0.0 && depth_m <= max_dist;
}

inline std::string formatSummary(double t, const Counts &c)
{
    char buf[160];
    std::snprintf(buf, sizeof(buf), "S %.6f %d %d %d %d %d", t, c.fixed, c.tri, c.rough, c.total,
                  c.flag1);
    return buf;
}

inline std::string formatAdopt(double t, int id, double depth_m)
{
    char buf[128];
    std::snprintf(buf, sizeof(buf), "A %.6f %d %.4f", t, id, depth_m);
    return buf;
}

inline std::string formatObs(double t, int id, double frame_stamp, double u, double v,
                             double depth_m, int verified_cnt)
{
    char buf[160];
    std::snprintf(buf, sizeof(buf), "O %.6f %d %.6f %.1f %.1f %.4f %d", t, id, frame_stamp, u, v,
                  depth_m, verified_cnt);
    return buf;
}

// 파일 기록기. 특징 id 는 창에서 사라질 때까지 flag 가 유지되므로 한 번만 A/O 를 남긴다.
// 메모리: 채택 id 집합이 프로세스 수명 동안 단조 증가한다(프로브 전용·기본 off 라 허용).
class Logger
{
public:
    // path 가 비어 있으면 비활성(모든 호출 no-op). 열기 실패는 stderr 로 알린다(오타로 조용히 꺼지는 것 방지).
    explicit Logger(const std::string &path)
    {
        if (path.empty())
            return;
        out_.open(path, std::ios::out | std::ios::trunc);
        if (!out_.is_open())
            std::fprintf(stderr, "[depth_adopt_probe] cannot open VINS_DEPTH_ADOPT_LOG=%s — probe disabled\n",
                         path.c_str());
    }
    bool enabled() const { return out_.is_open(); }

    // 처음 보는 고정 채택 특징이면 true(호출자가 O 행을 이어 쓴다).
    bool adopt(double t, int id, double depth_m)
    {
        if (!enabled() || !logged_.insert(id).second)
            return false;
        out_ << formatAdopt(t, id, depth_m) << '\n';
        return true;
    }
    void obs(double t, int id, double frame_stamp, double u, double v, double depth_m,
             int verified_cnt)
    {
        if (enabled())
            out_ << formatObs(t, id, frame_stamp, u, v, depth_m, verified_cnt) << '\n';
    }
    void summary(double t, const Counts &c)
    {
        if (enabled())
            out_ << formatSummary(t, c) << '\n' << std::flush;
    }
    // 추정기 재시작(clearState) 시 호출. id 는 프로세스 안에서 재사용되지 않지만, 재시작 후에도 추적이
    // 이어진 특징이 다시 채택되는 것을 새 이벤트로 남기기 위해 집합을 비운다.
    void reset() { logged_.clear(); }

private:
    std::ofstream out_;
    std::set<int> logged_;
};
}  // namespace depth_adopt_probe
