// [SW1-1889] depth 채택 프로브 — 분류·행 형식·중복 억제·비활성 no-op 단위 테스트
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "../src/utility/depth_adopt_probe.h"

using namespace depth_adopt_probe;

namespace
{
std::vector<std::string> readLines(const std::string &path)
{
    std::ifstream in(path);
    std::vector<std::string> lines;
    for (std::string l; std::getline(in, l);)
        lines.push_back(l);
    return lines;
}
}  // namespace

// flag 1 은 FIX_DEPTH 일 때만 fixed, 아니면 rough. flag 2 는 tri. flag1 은 FIX_DEPTH 와 무관하게 센다.
TEST(DepthAdoptProbe, CountClassifiesByFlagAndFixDepth)
{
    Counts c;
    count(1, true, c);
    count(1, false, c);
    count(2, true, c);
    count(0, true, c);
    EXPECT_EQ(c.fixed, 1);
    EXPECT_EQ(c.tri, 1);
    EXPECT_EQ(c.rough, 2);
    EXPECT_EQ(c.total, 4);
    EXPECT_EQ(c.flag1, 2);
}

// 관측 depth 는 (0, max] 만 후보. 0(무효)·초과는 제외, 경계값 max 는 포함.
TEST(DepthAdoptProbe, ObsHasDepthRange)
{
    EXPECT_FALSE(obsHasDepth(0.0, 2.5));
    EXPECT_TRUE(obsHasDepth(0.3, 2.5));
    EXPECT_TRUE(obsHasDepth(2.5, 2.5));
    EXPECT_FALSE(obsHasDepth(2.6, 2.5));
    EXPECT_FALSE(obsHasDepth(-1.0, 2.5));
}

// 행 형식이 오프라인 파서 규약(공백 구분, 접두 S/A/O, O 끝에 verified 0/1)과 일치한다.
TEST(DepthAdoptProbe, RowFormats)
{
    Counts c;
    c.fixed = 3;
    c.tri   = 4;
    c.rough = 5;
    c.total = 12;
    c.flag1 = 3;
    EXPECT_EQ(formatSummary(1.5, c), "S 1.500000 3 4 5 12 3");
    EXPECT_EQ(formatAdopt(1.5, 7, 1.23456), "A 1.500000 7 1.2346");
    EXPECT_EQ(formatObs(1.5, 7, 1.25, 10.04, 20.06, 0.5, true),
              "O 1.500000 7 1.250000 10.0 20.1 0.5000 1");
    EXPECT_EQ(formatObs(1.5, 7, 1.25, 10.04, 20.06, 0.5, false),
              "O 1.500000 7 1.250000 10.0 20.1 0.5000 0");
}

// 같은 id 는 A 행을 한 번만 남기고, reset 후에는 다시 남긴다(재시작 후 재채택을 새 이벤트로).
TEST(DepthAdoptProbe, LoggerWritesOncePerIdAndResets)
{
    const std::string path = ::testing::TempDir() + "depth_adopt_probe_test.log";
    {
        Logger lg(path);
        ASSERT_TRUE(lg.enabled());
        EXPECT_TRUE(lg.adopt(1.0, 7, 0.9));
        lg.obs(1.0, 7, 0.9, 1.0, 2.0, 0.9, true);
        EXPECT_FALSE(lg.adopt(1.1, 7, 0.9));  // 중복 억제
        lg.reset();
        EXPECT_TRUE(lg.adopt(1.2, 7, 0.9));
        Counts c;
        count(1, true, c);
        lg.summary(1.2, c);
    }
    const auto lines = readLines(path);
    ASSERT_EQ(lines.size(), 4u);
    EXPECT_EQ(lines[0].substr(0, 2), "A ");
    EXPECT_EQ(lines[1].substr(0, 2), "O ");
    EXPECT_EQ(lines[2].substr(0, 2), "A ");
    EXPECT_EQ(lines[3], "S 1.200000 1 0 0 1 1");
    std::remove(path.c_str());
}

// 경로가 비면 비활성: 어떤 호출도 파일을 만들지 않고 false 를 돌려준다.
TEST(DepthAdoptProbe, DisabledLoggerIsNoop)
{
    Logger lg("");
    EXPECT_FALSE(lg.enabled());
    EXPECT_FALSE(lg.adopt(1.0, 1, 1.0));
    Counts c;
    lg.summary(1.0, c);  // no-op, 크래시 없음
}

// 열 수 없는 경로면 비활성으로 떨어지되 크래시하지 않는다(경고는 stderr, 여기서는 동작만 확인).
TEST(DepthAdoptProbe, UnopenablePathDisablesWithoutCrash)
{
    Logger lg("/no/such/dir/depth_adopt_probe.log");
    EXPECT_FALSE(lg.enabled());
    EXPECT_FALSE(lg.adopt(1.0, 1, 1.0));
}
