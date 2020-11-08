#include <list>
#include <unordered_set>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include <Kin/frame.h>
#include <car_kinematic.h>
#include <approx_shape_to_sphere.h>
#include <graph_planner.h>
#include <komo_planner.h>
#include <markovian_tamp_controller.h>
#include <joint_path_tamp_controller.h>
#include <axis_bound.h>
#include <axis_distance.h>

//TODO : save skeleton (saveAll, possibility to give a full name)
// - check q and qdot in optimization levels
// refactor optimization levels
// output the q vectors
// try learn with deep network

//===========================================================================

void init(  const mp::TreeBuilder& tb, KOMO_ext * komo, int verbose )
{
  // ego car
  arr ego_start_speed{ 1.0, 0, 0 }; // 10 m/s
  komo->setVelocity( 0.0, 0.5, "car_ego", NULL, OT_sos, ego_start_speed );
  komo->addObjective( 0.0, -1.0, new CarKinematic( "car_ego", komo->world ), OT_eq, NoArr, 1e2, 1 );

  // car speeds
  arr desired_speed{ 1.0, 0, 0 };
  komo->setVelocity( 0.0, -1, "car_1", NULL, OT_eq, desired_speed );
  komo->setVelocity( 0.0, -1, "car_2", NULL, OT_eq, desired_speed );
  komo->setVelocity( 0.0, -1, "car_3", NULL, OT_eq, desired_speed );
  komo->setVelocity( 0.0, -1, "car_4", NULL, OT_eq, desired_speed );
  komo->setVelocity( 0.0, -1, "car_5", NULL, OT_eq, desired_speed );

//  const auto radius = 0.35;
//  komo->addObjective( 0.0, -1, new ApproxShapeToSphere( komo->world, "car_1", "car_2", radius ), OT_ineq );
//  komo->addObjective( 0.0, -1, new ApproxShapeToSphere( komo->world, "car_2", "car_3", radius ), OT_ineq );
//  komo->addObjective( 0.0, -1, new ApproxShapeToSphere( komo->world, "car_3", "car_4", radius ), OT_ineq );
//  komo->addObjective( 0.0, -1, new ApproxShapeToSphere( komo->world, "car_4", "car_5", radius ), OT_ineq );
}

void groundContinue( const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
}

void groundMergeBetween( const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  auto car_before = facts[0];
  auto car_next = facts[1];

  double phase = it.time.from;

  komo->addObjective( phase+0.8, -1, new AxisDistance( std::string("car_ego"),  std::string("lane_2"), 0.05, AxisDistance::Y, AxisDistance::MAX, AxisDistance::ABS ), OT_ineq );

  komo->addObjective( phase, -1, new AxisDistance( std::string("car_ego"), car_next, -1.2, AxisDistance::X, AxisDistance::MAX, AxisDistance::SIGNED ), OT_ineq );
  komo->addObjective( phase, -1, new AxisDistance( car_before, std::string("car_ego"), -1.2, AxisDistance::X, AxisDistance::MAX, AxisDistance::SIGNED ), OT_ineq );

  arr ego_desired_speed{ 1.0, 0, 0 }; // approx 50 kmh
  komo->setVelocity( phase+0.8, -1, "car_ego", NULL, OT_sos, ego_desired_speed );

//  komo->setPosition( phase+0.8, -1, "car_ego", car_next.c_str(), OT_sos, {-2.0, 0, 0} );
//  komo->setPosition( phase+0.8, -1, car_before.c_str(), "car_ego", OT_sos, {-2.0, 0, 0} );

//  const auto radius = 0.1;
//  komo->addObjective( phase, -1, new ApproxShapeToSphere( komo->world, "car_ego", car_next.c_str(),   radius ), OT_ineq );
//  komo->addObjective( phase, -1, new ApproxShapeToSphere( komo->world, car_before.c_str(), "car_ego", radius ), OT_ineq );

  //setKeepDistanceTask( phase+1, -1, komo, car_successors );
  //std::cout << "merge between " << car_before << " and " << car_next << std::endl;
}

std::vector< double > randomVector( uint dim )
{
  std::vector< double > vec( dim );

  for( auto & v : vec )
  {
    v = rnd.uni(-1.0, 1.0);
  }

  return vec;
}

void saveDataToFileveDataToFile( const std::string & outputFolderPath, const std::string & filename, const std::list< std::pair< std::vector< double >, Policy > > & deltasToSkeletons )
{
  std::ofstream of;
  of.open( outputFolderPath + "/" + filename );
  std::unordered_set< Policy, PolicyHasher > skeletons;

  // header
  const auto witnessSkeleton = deltasToSkeletons.begin()->second;
  const auto witnessQr = witnessSkeleton.qresult();
  of << "header_size" << ";" << 5 << std::endl; // 1
  of << "n_worlds" << ";" << witnessQr.nWorlds() << std::endl; // 2
  of << "q_mask" << ";"; // 3
  for( uint j = 0; j < witnessQr.qDim(); ++j )
  {
     of << witnessQr.qmask(j);
     if( j != witnessQr.qDim() - 1 )
     {
       of << ";";
     }
  }
  of << std::endl;
  of << "steps_per_phase" << ";" << witnessQr.stepsPerPhase() << std::endl; // 4

  // table header
  const auto delta = deltasToSkeletons.begin()->first; // 5
  for( uint i = 0; i < delta.size(); ++i )
  {
    of << "d" << i << ";";
  }
  of << "skeleton_hash";
  of << std::endl;

  // data
  for( const auto dataPair : deltasToSkeletons )
  {
    const auto delta = dataPair.first;
    const auto skeleton = dataPair.second;

    skeletons.insert( skeleton );

    // delta
    for( auto d : delta )
    {
      of << d << ";";
    }

    // skeleton data
    of << skeleton.hash();

    const auto qr = skeleton.qresult();
    for( uint w = 0; w < qr.nWorlds(); ++w )
    {
      for( uint s = 0; s < qr.nSteps(w); ++s )
      {
        const auto qvec = qr.q(w,s);

        for( auto q : qvec )
        {
          of << q;

          if( s != qr.nSteps(w) - 1 )
          {
            of << ";";
          }
        }
      }
    }

    of << std::endl;
  }

  of.close();

  // save all unique skeletons
  for( const auto & skeleton : skeletons )
  {
    skeleton.saveAll( outputFolderPath, "-" + std::to_string( skeleton.hash() ) );
  }
}

void plan( const std::string & outputFolderPath )
{
  std::list< std::pair< std::vector< double >, Policy > > deltasToSkeletons;

  for( uint i = 0; i < 1000; ++i )
  {
    std::cout << "*********************" << std::endl;
    std::cout << "*******"<< i << "******" << std::endl;
    std::cout << "*********************" << std::endl;

    matp::GraphPlanner tp;
    mp::KOMOPlanner mp;

    // set worlds
    mp.setKin( "LGP-merging-kin.g" );
    tp.setFol( "LGP-merging-1w.g" );

    // set planner specific parameters
    mp.setNSteps( 20 );
    mp.setSecsPerPhase( 1.0 );

    // register symbols
    mp.registerInit( init );
    mp.registerTask( "continue"        , groundContinue );
    mp.registerTask( "merge_between"   , groundMergeBetween );

    /// DECISION GRAPH
    tp.setR0( -0.01 );  // balance exploration
    tp.setMaxDepth( 7 );
    tp.buildGraph();

    //tp.saveGraphToFile( "graph.gv" );
    //generatePngImage( "graph.gv" );

    // set initial parameters
    auto vec = mp.drawRandomVector(); // random

    //auto vec = mp.drawRandomVector({0,0}); // middle // ok
    //auto vec = mp.drawRandomVector({1,1});   // middle
    //auto vec = mp.drawRandomVector({-1,-1}); // middle // ok
    //auto vec = mp.drawRandomVector({-1,1}); // middle
    //auto vec = mp.drawRandomVector({1,-1}); // rear //ok
    //auto vec = mp.drawRandomVector({0.5,-1}); // front

    //MarkovianTAMPController controller( tp, mp );
    JointPathTAMPController controller( tp, mp );

    TAMPlanningConfiguration conf;
    conf.maxIterations = 1000;
    conf.showFinalPolicy = true;
    conf.showDurationSecs = 30;
    conf.saveFinalPolicy = true;
    conf.saveInformedPolicy = true;
    auto policy = controller.plan(conf);

    deltasToSkeletons.push_back( std::make_pair( vec, policy ) );

    if( i && i % 100 == 0 )
    {
      saveDataToFileveDataToFile(outputFolderPath, "result-data-" + std::to_string(i) + ".csv", deltasToSkeletons);
    }
  }
  saveDataToFileveDataToFile(outputFolderPath, "result-data.csv", deltasToSkeletons);
}

std::list< std::vector< double > > parseDeltas( const std::string & filepath )
{
  std::list< std::vector< double > > deltas;

  std::ifstream infile;
  infile.open(filepath);

  std::string line;

  while ( std::getline(infile, line) )
  {
    char split_char = ';';
    std::istringstream iss(line);
    std::vector< double > delta;

    for ( std::string each; std::getline( iss, each, split_char ); delta.push_back( std::stod( each ) ) );

    delta.pop_back(); // remove last elements because it is the skeleton number

    deltas.push_back( delta );
  }

  return deltas;
}

std::unordered_set< Policy, PolicyHasher > parseSkeletons( const std::string & folderpath )
{
  std::unordered_set< Policy, PolicyHasher > skeletons;

  auto p = boost::filesystem::path( folderpath );

  for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator( p ) ) )
  {
    auto filepath = entry.path().string();

    if( filepath.find(".po") != std::string::npos )
    {
      Policy ske; ske.load( filepath );
      skeletons.insert( ske );
    }
  }

  return skeletons;
}

std::string getKinFilepath( const std::string & folderpath )
{
  auto p = boost::filesystem::path( folderpath );

  for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator( p ) ) )
  {
    auto filepath = entry.path().string();

    if( filepath.find("-kin.g") != std::string::npos )
    {
      return filepath;
    }
  }

  return "";
}

void saveSkeletonValuesToFile( const std::string filename, const std::map< std::vector< double >, std::unordered_set< Policy, PolicyHasher > > & deltasToValues )
{
  std::ofstream of;
  of.open( filename );

  for( const auto dataPair : deltasToValues )
  {
    const auto delta = dataPair.first;
    const auto skeletons = dataPair.second;

    for( auto d : delta )
    {
      of << d << ";";
    }

    of << "nan" << ";";

    for( const auto skeleton : skeletons )
    {
      //skeleton.saveAll( "-" + std::to_string(skeId) );
      of << skeleton.value() << ";";
    }

    of << std::endl;
  }

  of.close();
}

void evaluate_all_skeletons( const std::string & result_filepath  )
{
  // retrieve deltas
  auto deltas = parseDeltas( result_filepath );

  // retrieve skeletons
  const auto folderpath = boost::filesystem::path( result_filepath ).parent_path().string();
  auto skeletons = parseSkeletons( folderpath );

  // retrieve kin file
  auto kin_filepath = getKinFilepath( folderpath );

  // replan for each skeleton and each delta
  std::map< std::vector< double >, std::unordered_set< Policy, PolicyHasher > > deltasToValues;
  for( const auto & vec : deltas )
  {
    for( auto skeleton : skeletons )
    {
      // plan for each skeleton
      mp::KOMOPlanner mp;

      // set worlds
      mp.setKin( kin_filepath.c_str() );

      // set planner specific parameters
      mp.setNSteps( 20 );

      // register symbols
      mp.registerInit( init );
      mp.registerTask( "continue"        , groundContinue );
      mp.registerTask( "merge_between"   , groundMergeBetween );

      /// APPLY DELTAS
      mp.drawRandomVector( vec );

      /// MOTION PLANNING
      auto po     = MotionPlanningParameters( skeleton.id() );
      po.setParam( "type", "jointPath" );
      mp.solveAndInform( po, skeleton );

      deltasToValues[vec].insert( skeleton );
    }
  }

  saveSkeletonValuesToFile( folderpath + "/" + "result-values.csv", deltasToValues );
}

//===========================================================================

int main(int argc,char **argv)
{
  rai::initCmdLine(argc,argv);

  std::string outputFolderPath;
  if( argc <= 1 )
    outputFolderPath = std::string( argv[0] );
  else
    outputFolderPath = std::string( argv[1] );

  plan( outputFolderPath );
  //evaluate_all_skeletons("joint_car_kin/100/result-data-100.csv");

  return 0;
}
