#include <rrt_star.h>
#include <io.h>
#include <iostream>
#include <gtest/gtest.h>

TEST(RRTStarPlanner, Creation)
{
  SampleSpace<2> space(std::array<std::pair<double, double>, 2>{std::pair<double, double>{-1.0, 1.0},
                                                                std::pair<double, double>{-1.0, 1.0}});

  RRTStar<SampleSpace<2>> rrt(space);
}

TEST(RRTStarPlanner, PlanOnMap)
{
  SampleSpace<2> space(std::array<std::pair<double, double>, 2>{std::pair<double, double>{-1.0, 1.0},
                                                                std::pair<double, double>{-1.0, 1.0}});

  RRTStar<SampleSpace<2>> rrt(space, 0.1, 0.1);

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

  drawer.save("rrt_star_raod_map.pgm");
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
