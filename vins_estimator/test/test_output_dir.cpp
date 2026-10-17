// [SW1-1959] VINS 기록 폴더 규칙(output_dir.h) 단위 테스트
//   ① 환경변수가 이긴다(명시)  ② yaml output_path 가 비어 있지 않으면 그대로(명시)
//   ③ root + 시스템 자리 쓰기 가능 → /var/lib/edie/data/logs/vins 꼴(기본, 명시 아님)
//   ④ root 지만 시스템 자리를 못 씀 → 홈  ⑤ 비root → 홈(시스템 자리가 쓸 수 있어도)  ⑥ 아무것도 없음 → ""
//   ⑦ Prepare: 부모까지 만들고 쓸 수 있으면 true, 빈 경로·파일 자리면 false  ⑧ 쓰기 간격(처음 즉시, 10 s 안엔 안 씀)
#include <gtest/gtest.h>

#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

#include "../src/utility/output_dir.h"

namespace
{

// 시험마다 쓰는 임시 폴더(끝나면 지운다)
class TempDir
{
public:
  TempDir()
  {
    char tmpl[] = "/tmp/vins_output_dir_test.XXXXXX";
    const char * p = ::mkdtemp(tmpl);
    path_ = p ? p : "";
  }
  ~TempDir()
  {
    if (!path_.empty()) {
      const std::string cmd = "rm -rf '" + path_ + "'";
      (void)std::system(cmd.c_str());
    }
  }
  const std::string & path() const { return path_; }

private:
  std::string path_;
};

TEST(OutputDir, EnvWins)
{
  TempDir t;
  const auto r = vins_output_dir::ResolveFrom("/x/env", "/y/yaml", true, "/home/u", t.path());
  EXPECT_EQ(r.dir, "/x/env");
  EXPECT_TRUE(r.explicit_dir);
}

TEST(OutputDir, YamlExplicitWinsOverRule)
{
  TempDir t;
  const auto r = vins_output_dir::ResolveFrom(nullptr, "/y/yaml", true, "/home/u", t.path());
  EXPECT_EQ(r.dir, "/y/yaml");
  EXPECT_TRUE(r.explicit_dir);
  // 빈 환경변수는 없는 것과 같다
  const auto r2 = vins_output_dir::ResolveFrom("", "/y/yaml", false, nullptr, t.path());
  EXPECT_EQ(r2.dir, "/y/yaml");
}

TEST(OutputDir, RootWritableSystemRoot)
{
  TempDir t;
  const std::string sys = t.path() + "/var_lib_edie";  // 아직 없음 → root 판이 직접 만든다
  const auto r = vins_output_dir::ResolveFrom(nullptr, "", true, nullptr, sys);
  EXPECT_EQ(r.dir, sys + "/data/logs/vins");
  EXPECT_FALSE(r.explicit_dir);
}

TEST(OutputDir, RootButSystemUnusableFallsBackToHome)
{
  TempDir t;
  // 시스템 자리에 "파일" 을 두어 폴더를 못 만들게 한다(읽기 전용 파티션 흉내 — root 로 돌려도 막힘)
  const std::string sys = t.path() + "/not_a_dir";
  std::ofstream(sys) << "x";
  const auto r = vins_output_dir::ResolveFrom(nullptr, "", true, "/home/u", sys);
  EXPECT_EQ(r.dir, "/home/u/.edie/logs/vins");
  EXPECT_FALSE(r.explicit_dir);
}

TEST(OutputDir, NonRootUsesHomeEvenIfSystemWritable)
{
  TempDir t;
  const auto r = vins_output_dir::ResolveFrom(nullptr, "", false, "/home/edie", t.path());
  EXPECT_EQ(r.dir, "/home/edie/.edie/logs/vins");
  EXPECT_FALSE(r.explicit_dir);
}

TEST(OutputDir, NothingGivesEmpty)
{
  TempDir t;
  const auto r = vins_output_dir::ResolveFrom(nullptr, "", false, nullptr, t.path());
  EXPECT_TRUE(r.dir.empty());
  const auto r2 = vins_output_dir::ResolveFrom(nullptr, "", false, "", t.path());
  EXPECT_TRUE(r2.dir.empty());
}

TEST(OutputDir, PrepareCreatesNestedAndRejectsBad)
{
  TempDir t;
  const std::string nested = t.path() + "/a/b/c";
  EXPECT_TRUE(vins_output_dir::Prepare(nested));
  EXPECT_TRUE(vins_output_dir::IsWritableDir(nested));
  EXPECT_TRUE(vins_output_dir::Prepare(nested));  // 이미 있어도 true
  EXPECT_FALSE(vins_output_dir::Prepare(""));
  const std::string file = t.path() + "/file";
  std::ofstream(file) << "x";
  EXPECT_FALSE(vins_output_dir::Prepare(file));            // 파일 자리
  EXPECT_FALSE(vins_output_dir::Prepare(file + "/under"));  // 파일 아래는 못 만듦
}

// critic 1차 R1: 사람이 적은 경로는 마지막 폴더만 만든다 — 부모가 없으면 실패(옛 /tmp 설정이 eMMC 에 폴더를 새로 파지 않게)
TEST(OutputDir, PrepareExplicitCreatesOnlyLeaf)
{
  TempDir t;
  const std::string missing_parent = t.path() + "/no_parent/vins";
  EXPECT_FALSE(vins_output_dir::Prepare(missing_parent, true));
  EXPECT_FALSE(vins_output_dir::IsWritableDir(t.path() + "/no_parent"));  // 부모를 만들지 않았다
  const std::string leaf = t.path() + "/vins";
  EXPECT_TRUE(vins_output_dir::Prepare(leaf, true));   // 부모가 있으면 마지막 폴더는 만든다
  EXPECT_TRUE(vins_output_dir::Prepare(leaf, true));   // 이미 있어도 true
}

// critic 1차 N4: root 이고 HOME 이 있어도 시스템 자리가 이긴다
TEST(OutputDir, RootSystemWinsOverHome)
{
  TempDir t;
  const std::string sys = t.path() + "/var_lib_edie";
  const auto r = vins_output_dir::ResolveFrom(nullptr, "", true, "/home/root", sys);
  EXPECT_EQ(r.dir, sys + "/data/logs/vins");
}

TEST(OutputDir, WriteThrottle)
{
  vins_output_dir::WriteThrottle th(10.0);
  EXPECT_TRUE(th.Due(100.0));   // 처음은 바로
  EXPECT_FALSE(th.Due(100.1));  // 프레임마다 다시 쓰지 않음
  EXPECT_FALSE(th.Due(109.9));
  EXPECT_TRUE(th.Due(110.0));   // 10 s 지나면 다시
  EXPECT_FALSE(th.Due(115.0));
}

}  // namespace
