#include <functional>
#include <list>

#include <policy.h>
#include <policy_printer.h>

#include <policy_builder.h>

#include <mcts_planner.h>
#include <komo_planner.h>

#include <observation_tasks.h>
#include <approx_point_to_shape.h>
#include <vertical_velocity.h>
#include <axis_alignment.h>
#include <over_plane.h>

#include <node_visitors.h>

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

static void savePolicyToFile( const Policy::ptr & policy )
{
  std::stringstream namess;
  namess << "policy-" << policy->id() << ".gv";
  auto name = namess.str();

  std::ofstream file;
  file.open( name );
  PolicyPrinter printer( file );
  printer.print( policy );
  file.close();

  generatePngImage( name );
}

//==========Application specific grounders===================================
void groundPickUp( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  //disconnect object from table
  komo->setKinematicSwitch( t_end, true, "delete", "tableC", *symbols(0)  );
  //connect graspRef with object
  komo->setKinematicSwitch( t_end, true, "ballZero", "handL", *symbols(0)  );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": pick up " << *symbols(0) << " from " << *symbols(1) << std::endl;
  }
}

void groundUnStack( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  //disconnect object from table
  komo->setKinematicSwitch( t_end, true, "delete", *symbols(1), *symbols(0)  );
  //connect graspRef with object
  komo->setKinematicSwitch( t_end, true, "ballZero", "handL", *symbols(0)  );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": unstack " << *symbols(0) << " from " << *symbols(1) << std::endl;
  }
}

void groundPutDown( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //
  //std::cout << *symbols(0) << " place " << *symbols(1) << " on " << *symbols(2) << std::endl;

  if( *symbols(1) == "tableC_center" )
  {
    komo->setPlace( t_end, "handL", *symbols(0), "tableC", verbose );
  }
  else if( *symbols(1) == "tableC_left" )
  {
    komo->setPlace( t_end, "handL", *symbols(0), "tableC", verbose );
  }
  else if( *symbols(1) == "tableC_right" )
  {
    komo->setPlace( t_end, "handL", *symbols(0), "tableC", verbose );
  }
  else
  {
    CHECK( 0 , "" );
  }

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " put down " <<*symbols(0) << " at " << *symbols(1) << std::endl;
  }
}

void groundStack( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //
  //std::cout << *symbols(0) << " place " << *symbols(1) << " on " << *symbols(2) << std::endl;

  komo->setPlace( t_end, "handL", *symbols(0), *symbols(1), verbose );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " stack " <<*symbols(0) << " on " << *symbols(1) << std::endl;
  }
}


//void groundGetSight( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
//{
//  StringL symbols;
//  for(Node *p:n->parents) symbols.append(&p->keys.last());

//  double duration=n->get<double>();

//  //
//  const double t_start = phase;
//  const double t_end =   phase + duration;
//  //

//  rai::String arg = *symbols(0);

//  komo->setTask( t_start, t_end, new ActiveGetSight      ( "manhead",
//                                                                        arg,
//                                                                        //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
//                                                                        ARR( -0.0, 0.1, 0.4 ) ),  // pivot position  in container frame
//                OT_sos, NoArr, 1e2 );

//  komo->setTask( t_end-0.2, t_end, new ActiveGetSight      ( "manhead",
//                                                                        arg,
//                                                                        //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
//                                                                        ARR( -0.0, 0.1, 0.4 ) ),  // pivot position  in container frame
//                OT_ineq, NoArr, 1e2 );

//  if( verbose > 0 )
//  {
//    std::cout << t_start << "->" << t_end << ": getting sight of " << *symbols(0) << std::endl;
//  }
//}

//void groundTakeView( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
//{
//  StringL symbols;
//  for(Node *p:n->parents) symbols.append(&p->keys.last());

//  double duration=n->get<double>();

//  //
//  const double t_start = phase;
//  const double t_end =   phase + duration;
//  //

//  // no movement
//  auto *map = new TM_Transition( komo->world );
//  map->posCoeff = 0.;
//  map->velCoeff = 1.;
//  map->accCoeff = 0.;
//  komo->setTask( t_start, t_end, map, OT_sos, NoArr, 1e2, 1 );

//  // in sight expressed as a constraint
////  rai::String arg = *symbols(0);
////  komo->setTask( t_start, t_end, new ActiveGetSight      ( "manhead",
////                                                                        arg,
////                                                                        //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
////                                                                        ARR( -0.0, 0.1, 0.4 ) ),  // pivot position  in container frame
////                OT_eq, NoArr, 1e2 );

//  if( verbose > 0 )
//  {
//    std::cout << t_start << "->" << t_end << ": taking view " << std::endl;
//  }
//}

//void groundObjectPairCollisionAvoidance( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
//{
//  //std::cout << facts << std::endl;

//  StringL symbols;
//  for(Node *p:n->parents) symbols.append(&p->keys.last());

//  double duration=n->get<double>();

//  //
//  const double t_start = phase;
//  const double t_end =  komo->maxPhase;
//  //

//  for( auto s1 : komo->world.getBodyByName( *symbols(0) )->shapes )
//  {
//    for( auto s2 : komo->world.getBodyByName( *symbols(1) )->shapes )
//    {
//      //komo->setTask( t_start, t_end, new ShapePairCollisionConstraint( komo->world, s1->name, s2->name, 0.1 ), OT_ineq, NoArr, 1e2 );
//    }
//  }
//}

//===========================================================================

void plan()
{
  // instanciate planners
  auto tp = std::make_shared< tp::MCTSPlanner >();
  auto mp = std::make_shared< mp::KOMOPlanner >();

  // register symbols
  mp->registerTask( "komoPickUp"       , groundPickUp );
  mp->registerTask( "komoPutDown"      , groundPutDown );
  mp->registerTask( "komoStack"        , groundStack );
  mp->registerTask( "komoUnStack"      , groundUnStack );

  // set start configurations
  tp->setFol( "LGP-blocks-fol.g" );
  mp->setKin( "LGP-blocks-kin.g" );

  for( uint i = 0; ! tp->terminated() && i < 1 ; ++i )
  {
    std::cout << "Task planning to generate " << i << "th policy" << std::endl;

    // TASK PLANNING
    tp->solve();
    auto policy = tp->getPolicy();
    auto po     = tp->getPlanningOrder();

    // save policy
    savePolicyToFile( policy );

    std::cout << "Motion Planning for policy " << i << std::endl;

    // MOTION PLANNING
    mp->solveAndInform( po, policy );

    // print resulting cost
    std::cout << "cost of the policy " << i << " " << policy->cost() << std::endl;

    tp->integrate( policy );
  }

  std::cout << "best policy:" << tp->getPolicy()->id() << std::endl;
  mp->display( tp->getPolicy(), 3000 );
}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  plan();

  return 0;
}
