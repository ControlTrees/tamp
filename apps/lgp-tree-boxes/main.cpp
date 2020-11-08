#include <functional>
#include <list>
#include <chrono>
#include <fstream>

#include <graph_planner.h>
#include <komo_planner.h>

#include <observation_tasks.h>
#include <approx_point_to_shape.h>
#include <vertical_velocity.h>
#include <axis_alignment.h>
#include <over_plane.h>

#include <komo_planner.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>

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

static void savePolicyToFile( const Skeleton & policy, const std::string & suffix = "" )
{
  std::stringstream namess, skenamess;
  namess << "policy-" << policy.id() << suffix << ".gv";
  auto name = namess.str();

  policy.save( name );
  policy.saveToGraphFile( name );
  // generate nice graph
  //  {
  //    std::ofstream file;
  //    file.open( name );
  //    PolicyPrinter printer( file );
  //    printer.print( policy );
  //    file.close();

  //    generatePngImage( name );
  //  }
}

//==========Application specific grounders===================================
//------grounders------------//
void groundGrasp( double phase, const std::vector< std::string >& facts, mp::ExtensibleKOMO * komo, int verbose )
{
  double duration=1.0;

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  if( facts[1] == "container_0" )
  {
    //disconnect object from table
    komo->setKinematicSwitch( t_end, true, "delete", "tableC", "container_0_bottom" );
    //connect graspRef with object
    komo->setKinematicSwitch( t_end, true, "ballZero", facts[0].c_str(), "container_0_left" /**symbols(1)*/ );
  }
  else if( facts[1] == "container_1" )
  {
    //disconnect object from table
    komo->setKinematicSwitch( t_end, true, "delete", "tableC", "container_1_bottom" );
    //connect graspRef with object
    komo->setKinematicSwitch( t_end, true, "ballZero", facts[0].c_str(), "container_1_left" /**symbols(1)*/ );
  }

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": grasping " << facts[1] << " with " << facts[0] << std::endl;
  }
}

void groundGraspObject( double phase, const std::vector< std::string >& facts, mp::ExtensibleKOMO * komo, int verbose )
{
  double duration=1.0;

  //
  const double t_start = phase;
  const double t_approach=phase + 0.25 * duration;
  const double t_switch =phase  + 0.5 * duration;
  //const double t_escape =phase  + 0.75 * duration;
  const double t_end =   phase  + duration;
  //

  //komo->setTask( t_approach, t_approach, new TM_Default(TMT_pos, komo->world, *symbols(0), NoVector, *symbols(1), {0.,0.,0.5}), OT_sos, NoArr, 1e2);
  //komo->setTask( t_start,    t_approach, new TM_Default(TMT_pos,   komo->world, *symbols(0) ), OT_sos, {0.,0.,-.2}, 1e1, 1);

  //komo->setTask( t_switch, t_end, new TM_Default(TMT_pos, komo->world, *symbols(0), NoVector, *symbols(1), {0.,0.,0.1}), OT_sos, NoArr, 1e2);
  //komo->setTask( t_switch, t_end, new TM_Default(TMT_pos, komo->world, *symbols(0) ), OT_sos, {0.,0.,.2}, 1e1, 1);

  // approach
  //komo->setTask( t_start, t_approach, new TM_Default(TMT_pos, komo->world, *symbols(0), NoVector, *symbols(1), {0.,0.,0.4}), OT_sos, NoArr, 1e2);
  //komo->setTask( t_approach, t_end, new VerticalVelocity( *symbols(0), { 0.0,0.0 } ), OT_eq, NoArr, 1e1, 1 );

  //disconnect object from table
  komo->setKinematicSwitch( t_switch, true, "delete", NULL, facts[1].c_str() );
  //connect graspRef with object
  komo->setKinematicSwitch( t_switch, true, "ballZero", facts[0].c_str(), facts[1].c_str() );

  // escape
  //komo->setTask( t_escape, t_end, new TM_Default(TMT_pos, komo->world, *symbols(0), NoVector, *symbols(1), {0.,0.,0.5}), OT_sos, NoArr, 1e2);

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": grasping " << facts[1] << " with " << facts[1] << std::endl;
  }
}

void groundPlace( double phase, const std::vector< std::string >& facts, mp::ExtensibleKOMO * komo, int verbose )
{
  double duration=1.0;

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //
  //std::cout << *symbols(0) << " place " << *symbols(1) << " on " << *symbols(2) << std::endl;

  if( facts[1] == "container_0" )
  {
    komo->setPlace( t_end, facts[0].c_str(), "container_0_front", facts[2].c_str(), verbose );
  }
  else if( facts[1] == "container_1" )
  {
    komo->setPlace( t_end, facts[0].c_str(), "container_1_front", facts[2].c_str(), verbose );
  }

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << facts[0] << " place " << facts[1] << " on " << facts[2] << std::endl;
  }
}

//void groundHome( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
//{
//  double duration=n->get<double>();

//  const double t = phase+duration;

//  //komo->setHoming( t, t + 1.0, 1e-2 ); //gradient bug??
//}

void groundGetSight( double phase, const std::vector< std::string >& facts, mp::ExtensibleKOMO * komo, int verbose )
{
  double duration=1.0;

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  rai::String arg = facts[0].c_str();

  komo->setTask( t_start, t_end, new ActiveGetSight      ( "manhead",
                                                                        arg,
                                                                        //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
                                                                        ARR( -0.0, 0.1, 0.4 ), ARR( 0, -1, 0 ) ),  // pivot position  in container frame
                OT_sos, NoArr, 1e2 );

  komo->setTask( t_end-0.2, t_end, new ActiveGetSight      ( "manhead",
                                                                        arg,
                                                                        //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
                                                                        ARR( -0.0, 0.1, 0.4 ), ARR( 0, -1, 0 ) ),  // pivot position  in container frame
                OT_eq, NoArr, 1e2 );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": getting sight of " << facts[0] << std::endl;
  }
}

void groundTakeView( double phase, const std::vector< std::string >& facts, mp::ExtensibleKOMO * komo, int verbose )
{
  double duration=1.0;

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  // no movement
  auto *map = new TM_Transition( komo->world );
  map->posCoeff = 0.;
  map->velCoeff = 1.;
  map->accCoeff = 0.;
  komo->setTask( t_start, t_end, map, OT_sos, NoArr, 1e2, 1 );

  // in sight expressed as a constraint
//  rai::String arg = *symbols(0);
//  komo->setTask( t_start, t_end, new ActiveGetSight      ( "manhead",
//                                                                        arg,
//                                                                        //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
//                                                                        ARR( -0.0, 0.1, 0.4 ) ),  // pivot position  in container frame
//                OT_eq, NoArr, 1e2 );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": taking view " << std::endl;
  }
}

//class OverPlaneConstraintManager
//{
//public:

void groundActivateOverPlane( double phase, const std::vector< std::string >& facts, mp::ExtensibleKOMO * komo, int verbose )
{
  double duration=1.0;

  //
  const double t_start = phase;//phase + 1.0;       // hack: the grasp task lasts 1 step, so we begin one step after
  const double t_end =   phase + duration; //komo->maxPhase;
  //

  if( facts[0] == "container_0" )
  {
    komo->setTask( t_start, t_end, new AxisAlignment( "container_0", ARR( 0, 0, 1.0 ) ), OT_sos, NoArr, 1e2 );
    komo->setTask( t_start, t_end, new AxisAlignment( "container_0", ARR( 1.0, 0, 0 ) ), OT_sos, NoArr, 1e2 );

    auto task = komo->setTask( t_start, t_end, new OverPlaneConstraint( komo->world, "container_0", facts[1].c_str() , 0.05 ), OT_sos, NoArr, 1e2 );

    //activeTasks_.push_back( ActiveTask{ komo, symbols, task } );
  }
  else if( facts[0] == "container_1" )
  {
    komo->setTask( t_start, t_end, new AxisAlignment( "container_1", ARR( 0, 0, 1.0 ) ), OT_sos, NoArr, 1e2 );
    komo->setTask( t_start, t_end, new AxisAlignment( "container_1", ARR( 1.0, 0, 0 ) ), OT_sos, NoArr, 1e2 );

    auto task = komo->setTask( t_start, t_end, new OverPlaneConstraint( komo->world, "container_1", facts[1].c_str(), 0.05 ), OT_sos, NoArr, 1e2 );

    //activeTasks_.push_back( ActiveTask{ komo, symbols, task } );
  }

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": over plane of " << facts[0] << " activated" << std::endl;
  }
}

/*void groundDeactivateOverPlane( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  for( auto req : activeTasks_ )
  {
    if( req.komo == komo && req.symbols == symbols )
    {
      //req.task->
    }
  }
}
private:

struct ActiveTask
{
  KOMO * komo;
  StringL symbols;
  Task * task;
};

std::list< ActiveTask > activeTasks_;

};*/


void groundObjectPairCollisionAvoidance( double phase, const std::vector< std::string >& facts, mp::ExtensibleKOMO * komo, int verbose )
{
  double duration=1.0;

  //
  const double t_start = phase;
  const double t_end =  komo->maxPhase;
  //

//  for( auto s1 : komo->world.getBodyByName( *symbols(0) )->shapes )
//  {
//    for( auto s2 : komo->world.getBodyByName( *symbols(1) )->shapes )
//    {
//      //komo->setTask( t_start, t_end, new ShapePairCollisionConstraint( komo->world, s1->name, s2->name, 0.1 ), OT_ineq, NoArr, 1e2 );
//    }
//  }
}

//===========================================================================

void plan()
{
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

  // instanciate planners
  matp::GraphPlanner tp;
  mp::KOMOPlanner mp;

  // register symbols
  mp.registerTask( "grasp"       , groundGrasp );
  mp.registerTask( "graspObject" , groundGraspObject );
  mp.registerTask( "place"       , groundPlace );
  mp.registerTask( "get_sight"    , groundGetSight );
  mp.registerTask( "komoTakeView"    , groundTakeView );
  mp.registerTask( "komoActivateOverPlane"   , groundActivateOverPlane );
  //komoFactory_.registerTask( "komoDeactivateOverPlane" , groundDeactivateOverPlane );
  mp.registerTask( "komoCollisionAvoidance", groundObjectPairCollisionAvoidance );

  // set start configurations
  tp.setFol( "LGP-obs-container-fol-place-pick-2-no-take-view.g" );
  mp.setKin( "LGP-obs-container-kin.g" );

  tp.setR0( -0.0015 );
  tp.setMaxDepth( 20 );
  mp.setNSteps( 20 );

  {
    auto start = std::chrono::high_resolution_clock::now();
    /// GRAPH BUILDING
    tp.buildGraph(true);
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    graph_building_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
  }
  //tp.saveGraphToFile( "graph.gv" );
  //generatePngImage( "graph.gv" );

  Skeleton policy, lastPolicy;

  {
    auto start = std::chrono::high_resolution_clock::now();
    tp.solve();
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    task_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
  }
  policy = tp.getPolicy();

  do
  {
    ///
    savePolicyToFile( policy );
    candidate << policy.id() << "," << std::max( -10.0, policy.value() ) << std::endl;
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
    savePolicyToFile( policy, "-informed" );
    results << policy.id() << "," << std::max( -10.0, policy.value() ) << std::endl;
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

  } while( lastPolicy != policy );

  /////
  savePolicyToFile( policy, "-final" );
  candidate << policy.id() << "," << std::max( -10.0, policy.value() ) << std::endl;
  results << policy.id() << "," << std::max( -10.0, policy.value() ) << std::endl;

  candidate.close();
  results.close();
  /////
  {
    auto start = std::chrono::high_resolution_clock::now();
    mp.display( policy, 300 );
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    joint_motion_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
  }
  /////


  //    // eval
  //    auto eval = mp.evaluateLastSolution();
  //    acc_length( eval.first );
  //    acc_acc_cost( eval.second );

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

//void plan_graph()
//{
//  auto tp = std::make_shared< tp::GraphSearchPlanner >();
//  auto mp = std::make_shared< mp::KOMOPlanner >();

//  // set planner specific parameters
//  tp->setInitialReward( -0.1 );
//  mp->setNSteps( 5 );

//  tp->setFol( "LGP-obs-container-fol-place-pick-2-no-take-view.g" );
//  mp->setKin( "LGP-obs-container-kin.g" );

//  tp->buildGraph();

//  tp->saveGraphToFile( "graph.gv" );
//  generatePngImage( "graph.gv" );
//}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  plan();

  return 0;
}
