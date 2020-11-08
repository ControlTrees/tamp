#include <functional>
#include <list>

#include <graph_planner.h>

#include <komo_planner.h>

#include <axis_bound.h>

#include <komo_wrapper.h>

//===========================================================================

using W = mp::KomoWrapper;

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

void init( const mp::TreeBuilder& tb, KOMO_ext * komo, int verbose  )
{
  // road bounds
  komo->addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo->world ), OT_ineq, {-0.15} );
  komo->addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MAX, komo->world ), OT_ineq, { 0.15} );

  // min speed
  komo->addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::X, AxisBound::MIN, komo->world ), OT_ineq, - arr{0.03}, 1e2, 1 );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  truck_speed( 0 ) = 0.03;
  komo->setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );
  //komo->setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );
  //komo->setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );


  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
  op_speed( 0 ) = -0.03;
  komo->setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

  komo->activateCollisions( "car_ego", "truck" );
  komo->activateCollisions( "car_ego", "car_op" );

  // min speed
  komo->addObjective( 0.0, 1.0, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MAX, komo->world ), OT_sos, arr{-0.1} );
  komo->addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::X, AxisBound::MIN, komo->world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // collision
  komo->activateCollisions( "car_ego", "truck" );
  komo->activateCollisions( "car_ego", "car_op" );
  komo->add_collision( true );
}

void groundLook( const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  //
  const double t_start = it.time.from;
  const double t_end =   it.time.to;
  //

  // look
  komo->addObjective( t_start + 0.9, t_end, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo->world ), OT_sos );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " look " << facts[0] << " at " << facts[1] << std::endl;
  }
}

void groundOvertake( const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  double duration=1.0;

  //
  const double t_start = it.time.from;
  const double t_end =   it.time.to;
  //

  // overtake
  //komo->setTask( t_start -0.5, t_start + 0.5, new AxisBound( "car_ego", 0.05, AxisBound::Y, AxisBound::MIN ), OT_sos );
  komo->setPosition( t_end, -1, "car_ego", facts[0].c_str(), OT_sos, { 0.45, 0, 0 } );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " overtake " << facts[0] << std::endl;
  }
}

void groundFollow( const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  //
  const double t_start = it.time.from;
  const double t_end =   it.time.to;
  //

  // overtake
  komo->setPosition( t_end, -1, "car_ego", "truck", OT_sos, { -0.7, 0, 0 } ); // -0.55

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " follow " << facts[0] << " at " << facts[1] << std::endl;
  }
}

//===========================================================================

void plan_graph_search()
{
  matp::GraphPlanner tp;
  mp::KOMOPlanner mp;

  // set planner specific parameters
  mp.setNSteps( 20 );

  // register symbols
  mp.registerInit( init );
  mp.registerTask( "look"      , groundLook );
  mp.registerTask( "overtake"  , groundOvertake );
  mp.registerTask( "follow"    , groundFollow );


  // set start configurations
  //mp.setKin( "LGP-overtaking-kin-2w_bis.g" );
  //tp.setFol( "LGP-overtaking-2w.g" );
  //tp.setFol( "LGP-overtaking-2w.g" );

  //mp.setKin( "LGP-overtaking-kin-3w.g" ); // needs another init!!
  //tp.setFol( "LGP-overtaking-3w.g" );

  mp.setKin( "LGP-overtaking-kin-1w.g" ); // needs another init!!
  tp.setFol( "LGP-overtaking-1w.g" );

  /// DECISION GRAPH
  tp.setR0( -0.001 );  // balance exploration
  tp.buildGraph();

  tp.saveGraphToFile( "graph.gv" );
  generatePngImage( "graph.gv" );

  /// LOOP
  Policy policy, lastPolicy;
  tp.solve();
  policy = tp.getPolicy();

  uint nIt = 0;
  const uint maxIt = 1000;
  do
  {
    nIt++;

    lastPolicy = policy;

    /// MOTION PLANNING
    auto po     = MotionPlanningParameters( policy.id() );
    po.setParam( "type", "markovJointPath" );
    mp.solveAndInform( po, policy );

    ///
    savePolicyToFile( policy, "-informed" );
    ///

    /// TASK PLANNING
    tp.integrate( policy );
    tp.solve();

    policy = tp.getPolicy();
  }
  while( lastPolicy != policy && nIt != maxIt );
  ///

  savePolicyToFile( policy, "-final" );
  mp.display( policy, 3000 );

  rai::wait( 30, true );
}

//===========================================================================
void groundSpeed1( const mp::Interval& it, const mp::TreeBuilder& tree, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  arr speed{ 0.03, 0, 0 };
  speed( 0 ) = 0.10;
  W(komo).addObjective(it, tree, new TM_Default(TMT_pos, komo->world, "car_ego"), OT_eq, speed, 1e1, 1);
  if( verbose > 0 )
  {
    std::cout << it.edge.from << "->" << it.edge.to << ": " << " speed1 " << facts[0] << std::endl;
  }
}

void groundSpeed2( const mp::Interval& it, const mp::TreeBuilder& tree, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  arr speed{ 0.03, 0, 0 };
  speed( 0 ) = 0.02;
  W(komo).addObjective(it, tree, new TM_Default(TMT_pos, komo->world, "car_ego"), OT_eq, speed, 1e1, 1);
  if( verbose > 0 )
  {
    std::cout << it.edge.from << "->" << it.edge.to << ": " << " speed2 " << facts[0] << std::endl;
  }
}

void minimal_plan_admm()
{
  matp::GraphPlanner tp;
  mp::KOMOPlanner mp;

  // set planner specific parameters
  mp.setNSteps( 50 );

  // register symbols
  mp.registerTask( "speed1"  , groundSpeed1 );
  mp.registerTask( "speed2"  , groundSpeed2 );
  mp.registerTask( "speed3"  , groundSpeed1 );
  mp.registerTask( "speed4"  , groundSpeed2 );
  mp.registerTask( "speed5"  , groundSpeed1 );
  mp.registerTask( "speed6"  , groundSpeed2 );
  mp.registerTask( "speed7"  , groundSpeed1 );
  mp.registerTask( "speed8"  , groundSpeed2 );
  mp.registerTask( "speed9"  , groundSpeed1 );
  mp.registerTask( "speed10"  , groundSpeed2 );

  mp.setKin( "LGP-speed-kin-1w.g" ); // needs another init!!
  tp.setFol( "LGP-speed-1w.g" );

  /// DECISION GRAPH
  tp.setMaxDepth(10);
  tp.setR0( -0.001 );  // balance exploration
  tp.buildGraph();

  tp.saveGraphToFile( "graph.gv" );
  generatePngImage( "graph.gv" );

  /// LOOP
  Policy policy, lastPolicy;
  tp.solve();
  policy = tp.getPolicy();
  savePolicyToFile( policy, "-" );

  /// MOTION PLANNING
  auto po     = MotionPlanningParameters( policy.id() );

  //po.setParam( "type", "jointSparse" );
  po.setParam( "type", "ADMMCompressed" );
  po.setParam( "decompositionStrategy", "LinearSplit" ); // ENABLE HERE AND PUT LINEAR TRAJ TO CONTINUE WORKING
  po.setParam( "nJobs", "8" );

  mp.solveAndInform( po, policy );
}

//===========================================================================

int main(int argc,char **argv)
{
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  //plan_graph_search();
  minimal_plan_admm();

  return 0;
}
