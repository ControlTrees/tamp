#include <rrt.h>
#include <tree_drawer.h>
#include <iostream>
#include <gtest/gtest.h>

TEST(SampleSpace, SampleBounds)
{
  SampleSpace<2> space(std::array<std::pair<double, double>, 2>{std::pair<double, double>{-1.0, 1.0}, std::pair<double, double>{-1.0, 1.0}});

  for(auto i = 0; i < 100; ++i)
  {
    auto s = space.sample();

    //
    //std::cout << s[0] << " " << s[1] << std::endl;
    //

    EXPECT_GE(s[0], -1.0);
    EXPECT_LE(s[0], 1.0);

    EXPECT_GE(s[1], -1.0);
    EXPECT_LE(s[1], 1.0);
  }
}

TEST(RRTPlanner, creation)
{
  SampleSpace<2> space(std::array<std::pair<double, double>, 2>{std::pair<double, double>{-1.0, 1.0},
                                                                std::pair<double, double>{-1.0, 1.0}});

  RRT<SampleSpace<2>> rrt(space);
}

TEST(RRTPlanner, PlanFreeSpace)
{
  SampleSpace<2> space(std::array<std::pair<double, double>, 2>{std::pair<double, double>{-1.0, 1.0},
                                                                std::pair<double, double>{-1.0, 1.0}});

  RRT<SampleSpace<2>> rrt(space);

  auto state_checker = [](const std::array<double, 2> & state) -> bool
  {
    return true;
  };

  auto goal_cnd = [](const std::array<double, 2> & state) -> bool
  {
    return fabs(state[0] - 0.9) < 0.05 && fabs(state[1] - 0.9) < 0.05;
  };

  auto transition = [](const std::array<double, 2> & from, const std::array<double, 2> & to) -> bool
  {
    return true;
  };

  rrt.set_state_checker(state_checker);
  rrt.set_transition_checker(transition);

  auto path = rrt.plan({0.0, 0.0}, goal_cnd, 1000);

  for(auto s: path)
  {
    std::cout << s[0] << " " << s[1] << std::endl;
  }

  EXPECT_GE(path.size(), 2);
  EXPECT_TRUE(goal_cnd(path.back()));
}

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

TEST(RRTPlanner, PlanOnMap)
{
  SampleSpace<2> space(std::array<std::pair<double, double>, 2>{std::pair<double, double>{-1.0, 1.0},
                                                                std::pair<double, double>{-1.0, 1.0}});

  RRT<SampleSpace<2>> rrt(space, 0.1);

  MapLoader map("data/map0.pgm", space.bounds());
  auto state_checker = [&map](const std::array<double, 2> & state) -> bool
  {
    return map.is_state_valid(state);
  };

  auto goal_cnd = [](const std::array<double, 2> & state) -> bool
  {
    return fabs(state[0] - 0.0) < 0.05 && fabs(state[1] - 0.9) < 0.05;
  };

  auto transition = [](const std::array<double, 2> & from, const std::array<double, 2> & to) -> bool
  {
    return true;
  };

  rrt.set_state_checker(state_checker);
  rrt.set_transition_checker(transition);

  auto path = rrt.plan({0.0, 0.0}, goal_cnd, 5000);

  Drawer drawer(space.bounds(), 100);

  drawer.draw_tree(rrt.rrt_tree());
  drawer.draw_path(path);

  drawer.save("SaveRoadMap.pgm");
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
