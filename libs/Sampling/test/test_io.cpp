#include <io.h>
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

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
