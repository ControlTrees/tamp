#include <functional>
#include <chrono>
#include <deque>
#include <list>

#include <KOMO/komo.h>
#include <gtest/gtest.h>

#include <car_kinematic.h>
#include <velocity.h>
#include <occupancy_grid.h>

//using namespace std;

/////////////////////////////////////
void init_from_pose(double x, double y, double yaw,  KOMO & komo)
{
  komo.world.q(0)=x;
  komo.world.q(1)=y;
  komo.world.q(2)=yaw;
  komo.world.calc_Q_from_q();
  komo.world.calc_fwdPropagateFrames();
  komo.reset();
}

//void init_with_last_traj(const std::deque<arr> & traj_starts, const arr & last_x, KOMO & komo, bool show)
//{
//  if(!last_x.d0)
//  {
//    init_from_pose(traj_starts.front()(0), traj_starts.front()(1), traj_starts.front()(2), komo);
//    return;
//  }

//  // init world
//  komo.world.q(0)=traj_starts[traj_starts.size()-2](0);
//  komo.world.q(1)=traj_starts[traj_starts.size()-2](1);
//  komo.world.q(2)=traj_starts[traj_starts.size()-2](2);
//  komo.world.calc_Q_from_q();
//  komo.world.calc_fwdPropagateFrames();

//  // init traj
//  arr new_x = last_x;
//  const uint n_steps_ahead = 1;
//  for(auto s=0; s < last_x.d0 - n_steps_ahead*komo.world.q.d0; ++s)
//  {
//    new_x(s) = last_x(s+n_steps_ahead*komo.world.q.d0);
//  }

//  komo.set_x(new_x); // set-up prefix and normal frames

//  // set middle frame
//  komo.configurations(1)->q(0)=traj_starts[traj_starts.size()-1](0);
//  komo.configurations(1)->q(1)=traj_starts[traj_starts.size()-1](1);
//  komo.configurations(1)->q(2)=traj_starts[traj_starts.size()-1](2);
//  komo.configurations(1)->calc_Q_from_q();
//  komo.configurations(1)->calc_fwdPropagateFrames();

//  komo.reset();
//}

//void print_configurations(const KOMO & komo)
//{
//  for(auto i = 0; i < komo.configurations.d0; ++i)
//  {
//    std::cout << komo.configurations(i)->q(0) << " " << komo.configurations(i)->q(1) << std::endl;
//  }
//}


TEST(KOMO_realtime, box)
{
  std::list< std::string > maps;
  maps.push_back("data/sensor_map_corridor_oblique.png");
  maps.push_back("data/sensor_map_corridor.png");
  maps.push_back("data/sensor_map_rect_500.png");
  maps.push_back("data/sensor_map_circle_500.png");

  rai::KinematicWorld kin( "data/LGP-real-time.g" );
  const uint n_executed_steps = 1; //50;
  const uint n_steps_per_phases = 5;

  for( auto map: maps )
  {
    // run
    std::deque< arr > first_traj_poses;
    first_traj_poses.push_back({kin.q(0), kin.q(1), kin.q(2)});

    auto * vel = new Velocity("car_ego", 0.5);
    auto * car_kin = new CarKinematic("car_ego", kin);
    auto * circular_cage = new CircularCage("car_ego", {0.0, 0.0}, 2.5);
    auto * ocg = new OccupancyGrid("car_ego");
    ocg->setMapFromFile(map);

    for(auto i = 0; i < n_executed_steps; ++i)
    {
      // create komo
      KOMO komo;
      komo.setModel( kin, false );
      komo.setTiming(3.0, n_steps_per_phases, 1);

      // optimization objectives
      komo.setSquaredQAccelerations();
      komo.addObjective(0, -1, vel, OT_sos, NoArr, 1e2, 1);
      komo.addObjective(0, -1, car_kin, OT_eq, NoArr, 1e2, 1);
      //komo.setTask(0, -1, circular_cage, OT_ineq, NoArr, 1e2, 1);
      komo.addObjective(0, -1, ocg, OT_ineq, NoArr, 1e2, 1);

      // set start kin
      auto pose = first_traj_poses.back();
      init_from_pose(pose(0), pose(1), pose(2), komo);

      komo.world.watch();
      //komo.checkGradients();
      //continue;
      //DEBUG
      //std::cout << "init:" << std::endl;
      //print_configurations(komo);

      // run
      auto start = std::chrono::high_resolution_clock::now();
      komo.run();
      auto end = std::chrono::high_resolution_clock::now();
      auto execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

      auto x = komo.configurations(2)->q(0);
      auto y = komo.configurations(2)->q(1);
      auto yaw = komo.configurations(2)->q(2);
      first_traj_poses.push_back({x, y, yaw});

      // ASSERT NO COLLISIONS
      EXPECT_TRUE( ocg->get_distance_info({x, y})[0] > 0.0 );

      // DEBUG
      //print_configurations(komo);
      std::cout << "optimization time (ms):" << execution_time_us / 1000 << std::endl;
      //    komo.getReport(true);
      //komo.displayTrajectory(0.1);
      //
    }
  }
}

TEST(KOMO_realtime, circle_caging)
{
  rai::KinematicWorld G( "data/LGP-real-time.g" );

  auto * circular_cage = new CircularCage("car_ego", {0.0, 0.0}, 2.5);
  auto * ocg = new OccupancyGrid("car_ego");

  ocg->setMapFromFile("data/sensor_map_circle_500.png");

  std::list< arr > test_poses;

  test_poses.push_back({0.0, 0.0, 0.0});
  test_poses.push_back({0.5, 0.5, 0.0});
  test_poses.push_back({1.5, 0.5, 0.0});
  test_poses.push_back({0.0, 5.5, 0.0});
  test_poses.push_back({3.5, 0.0, 0.0});

  for(auto pose : test_poses)
  {
    G.q(0) = pose(0);
    G.q(1) = pose(1);
    G.q(2) = pose(2);
    G.calc_Q_from_q();
    G.calc_fwdPropagateFrames();

    arr y_cage;
    arr J_cage;
    circular_cage->phi(y_cage, J_cage, G);

    arr y_ocg {pose(0), pose(1), pose(2)};
    arr J_ocg;
    ocg->phi(y_ocg, J_ocg, G);

    auto diff = y_cage - y_ocg;
    auto J_diff = J_cage - J_ocg;

    const double tolerance = 0.15;
    EXPECT_NEAR(y_cage(0), y_ocg(0), tolerance);
    for(auto j = 0; j < J_cage.N; ++j)
    {
      EXPECT_NEAR(J_ocg.data()[j], J_cage.data()[j], tolerance);
    }
  }
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

