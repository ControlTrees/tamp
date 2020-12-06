#include <common.h>
#include <gtest/gtest.h>

TEST(PathCost, PathLength)
{
  //double get_cost(const std::deque<std::array<double, N>> & path)
  std::deque<std::array<double, 2>> path;
  path.push_back({0, 0});
  path.push_back({1, 1});
  path.push_back({2, 1});

  const auto l = get_cost(path);

  EXPECT_NEAR(l, sqrt(2) + 1, 1e-5);
}

TEST(Backtrack, NoBacktrackIfDistanceSmall)
{
  std::array<double, 2> from{0, 0};
  std::array<double, 2> to{1, 1};

  backtrack(from, to, 2.0);

  EXPECT_EQ(to[0], 1);
  EXPECT_EQ(to[1], 1);
}

TEST(Backtrack, BacktrackIfHighDistance)
{
  std::array<double, 2> from{0, 0};
  std::array<double, 2> to{1, 1};

  backtrack(from, to, 0.5);

  EXPECT_NEAR(to[0], 0.25, 1e-5); // use L1
  EXPECT_NEAR(to[1], 0.25, 1e-5);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
