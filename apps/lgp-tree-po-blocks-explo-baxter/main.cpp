#include <functional>
#include <list>
#include <chrono>

#include <boost/filesystem.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <Kin/kinViewer.h>

#include <graph_planner.h>

#include <komo_planner.h>
#include <approx_shape_to_sphere.h>
#include <observation_tasks.h>

#include "komo_tree_groundings.h"


//===========================================================================

static void generatePngImage( const std::string & name )
{
  std::string nameCopy( name );
  const std::string ext( ".gv" );
  std::string newName = nameCopy.replace( nameCopy.find( ext ), ext.length(), ".png" );

  std::stringstream ss;
  ss << "dot"   << " ";
  ss << "-Tpng" << " ";
  ss << "-o"    << " ";
  ss << newName << " ";
  ss << name;

  system( ss.str().c_str() );
}

static void savePolicyToFile( const Policy & policy, const std::string & suffix = "" )
{
  std::stringstream namess, skenamess;

  namess << "policy-" << policy.id() << suffix;
  policy.save( namess.str() );

  namess << ".gv";
  policy.saveToGraphFile( namess.str() );
}

//===========================================================================

void plan_graph_search()
{
  ///
  std::ofstream candidate, results;
  candidate.open( "policy-candidates.data" );
  results.open( "policy-results.data" );
  double graph_building_s = 0;
  double task_planning_s = 0;
  double motion_planning_s = 0;
  double joint_motion_planning_s = 0;

  namespace ba = boost::accumulators;
  boost::accumulators::accumulator_set<double, ba::features< ba::tag::variance, ba::tag::mean, ba::tag::min, ba::tag::max > > acc_length;
  boost::accumulators::accumulator_set<double, ba::features< ba::tag::variance, ba::tag::mean, ba::tag::min, ba::tag::max > > acc_acc_cost;

  for( auto i = 0; i < 1; ++i )
  {
    matp::GraphPlanner tp;
    mp::KOMOPlanner mp;

    // set planner specific parameters
    tp.setR0( -0.25 ); //-0.25//-0.1//-0.015 );
    tp.setMaxDepth( 20 );
    mp.setNSteps( 20 );
    mp.setMinMarkovianCost( 0.00 );
    ///
    // register symbols
    mp.registerInit( groundTreeInit );
    mp.registerTask( "pick-up"      , groundTreePickUp );
    mp.registerTask( "put-down"     , groundTreePutDown );
    mp.registerTask( "check"        , groundTreeCheck );

    // C
    tp.setFol( "LGP-blocks-fol-one-block.g" );
    mp.setKin( "LGP-blocks-kin-one-block.g" );

    {
      auto start = std::chrono::high_resolution_clock::now();
      /// GRAPH BUILDING
      tp.buildGraph(true);
      auto elapsed = std::chrono::high_resolution_clock::now() - start;
      graph_building_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
    }
    //tp.saveGraphToFile( "graph.gv" );

    Policy policy, lastPolicy;

    {
      auto start = std::chrono::high_resolution_clock::now();
      tp.solve();
      auto elapsed = std::chrono::high_resolution_clock::now() - start;
      task_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
    }
    policy = tp.getPolicy();

    uint nIt = 0;
    const uint maxIt = 1000;
    do
    {
      nIt++;
      ///
      savePolicyToFile( policy );
      candidate << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;
      ///

      lastPolicy = policy;

      {
        auto start = std::chrono::high_resolution_clock::now();
        /// MOTION PLANNING
        auto po     = MotionPlanningParameters( policy.id() );
        po.setParam( "type", "markovJointPath" );
        mp.solveAndInform( po, policy );
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        motion_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
      }
      ///
      //savePolicyToFile( policy, "-informed" );
      //results << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;
      ///

      {
        auto start = std::chrono::high_resolution_clock::now();
        /// TASK PLANNING
        tp.integrate( policy );
        tp.solve();
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        task_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
      }
      policy = tp.getPolicy();

    } while( lastPolicy != policy && nIt != maxIt );

    /////
    savePolicyToFile( policy, "-final" );
    candidate << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;
    results << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;

    candidate.close();
    results.close();
    /////
    {
      auto start = std::chrono::high_resolution_clock::now();
      mp.display( policy, 3000 );
      auto elapsed = std::chrono::high_resolution_clock::now() - start;
      joint_motion_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
    }
    /////

//    // eval
//    auto eval = mp.evaluateLastSolution();
//    acc_length( eval.first );
//    acc_acc_cost( eval.second );
  }

  std::ofstream timings;
  timings.open("timings.data");
  timings << "graph_building_s="<< graph_building_s << std::endl;
  timings << "task_planning_s="<< task_planning_s << std::endl;
  timings << "motion_planning_s="<< motion_planning_s << std::endl;
  timings << "joint_motion_planning_s="<< joint_motion_planning_s << std::endl;
  timings << "total_s="<< graph_building_s + task_planning_s + motion_planning_s + joint_motion_planning_s << std::endl;

  timings.close();

  // evaluation
  std::cout << "LENGTH: [" <<  ba::min( acc_length ) << " " << ba::max( acc_length ) << "] mean:" << ba::mean( acc_length ) << " std_dev:" << sqrt( ba::variance( acc_length ) ) << std::endl;
  std::cout << "ACC COSTS: [" << ba::min( acc_acc_cost ) << " " << ba::max( acc_acc_cost ) << "] mean:" << ba::mean( acc_acc_cost ) << " std_dev:" << sqrt( ba::variance( acc_acc_cost ) ) << std::endl;
}

void komo_tree_dev()
{
  mp::KOMOPlanner mp;

  // set planner specific parameters
  mp.setNSteps( 20 );
  mp.setMinMarkovianCost( 0.00 );
  mp.setExecutionPolicy(std::launch::async);

  // register symbols
  mp.registerInit( groundTreeInit );
  mp.registerTask( "pick-up"      , groundTreePickUp );
  mp.registerTask( "put-down"     , groundTreePutDown );
  mp.registerTask( "check"        , groundTreeCheck );

  mp.setKin( "LGP-blocks-kin-one-block.g" );

  // load policy
  Policy policy;
  policy.load("policy-0-2w");
  //policy.load("policy-0-6w");
  //policy.load("policy-0");

  // plan
  auto po     = MotionPlanningParameters( policy.id() );
  //po.setParam( "type", "jointSparse" );
  //po.setParam( "type", "ADMMSparse" );
  po.setParam( "type", "ADMMCompressed" );

  mp.solveAndInform( po, policy );
  //mp.display(policy, 200);
}


//===========================================================================

int main(int argc,char **argv)
{
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  komo_tree_dev();

  //plan_graph_search();

  return 0;
}
