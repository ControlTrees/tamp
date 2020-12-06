#include <io.h>
#include <sample_space.h>
#include <gtest/gtest.h>

TEST(RRTPlanner, ParseMap)
{
  SampleSpace<2> space(std::array<std::pair<double, double>, 2>{std::pair<double, double>{-1.0, 1.0},
                                                                std::pair<double, double>{-1.0, 1.0}});

  MapLoader map("data/map0.pgm", space.bounds());

  EXPECT_FALSE(map.is_state_valid({0, 0.6}));
  EXPECT_TRUE(map.is_state_valid({0, 0}));
  EXPECT_TRUE(map.is_state_valid({0.8, 0.2}));
  EXPECT_TRUE(map.is_state_valid({0.8, 0.1}));
}

TEST(RRTPlanner, MultiMap)
{
  SampleSpace<2> space(std::array<std::pair<double, double>, 2>{std::pair<double, double>{-1.0, 1.0},
                                                                std::pair<double, double>{-1.0, 1.0}});

  MultiMap<2> map({"data/map0.pgm", "data/map1.pgm"}, space.bounds());

  EXPECT_EQ(map.is_state_valid({0, 0.6})[0], 0);
  EXPECT_EQ(map.is_state_valid({0, 0.6})[1], 0);

  EXPECT_EQ(map.is_state_valid({0.3, 0.6})[0], 1);
  EXPECT_EQ(map.is_state_valid({0.3, 0.6})[1], 0);

  EXPECT_EQ(map.is_state_valid({0.0, 0.0})[0], 1);
  EXPECT_EQ(map.is_state_valid({0.0, 0.0})[1], 1);
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
