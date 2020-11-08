#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_graph_build
#include <boost/test/unit_test.hpp>

#include <chrono>
#include <unordered_set>

#include <graph_search.h>
#include <policy_printer.h>

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
  if( ! policy )
    return;

  std::stringstream namess, skenamess;
  namess << "policy-" << policy->id() << ".gv";
  skenamess << "policy-" << policy->id() << ".ske";
  auto skename = skenamess.str();
  auto name = namess.str();

  // save full policy
//  {
//    std::ofstream file;
//    file.open( skename );
//    policy->save( file );
//    file.close();
//  }
  // generate nice graph
  {
    std::ofstream file;
    file.open( name );
    PolicyPrinter printer( file );
    printer.print( policy );
    file.close();

    generatePngImage( name );
  }
}

BOOST_AUTO_TEST_CASE( test_LGP_boxes )
{
  // tests that Dijkstra works well from root and from other nodes
  auto tp = std::make_shared< tp::GraphSearchPlanner >();
  tp->setFol( "data/LGP-obs-container-fol-simple.g" );
  tp->buildGraph();
  tp->saveGraphToFile( "graph.gv" );
  generatePngImage( "graph.gv" );
}

//BOOST_AUTO_TEST_CASE( test_LGP_blocks_fol_easy_1w )
//{
//  // tests that Dijkstra works well from root and from other nodes
//  auto tp = std::make_shared< tp::GraphSearchPlanner >();
//  tp->setFol( "data/LGP-blocks-fol-easy-1w.g" );
//  tp->buildGraph();
//  //tp->saveGraphToFile( "graph.gv" );
//  //generatePngImage( "graph.gv" );
//}

//BOOST_AUTO_TEST_CASE( test_LGP_blocks_fol_easy_2w )
//{
//  // tests that Dijkstra works well from root and from other nodes
//  auto tp = std::make_shared< tp::GraphSearchPlanner >();
//  tp->setFol( "data/LGP-blocks-fol-easy-2w.g" );
//  tp->buildGraph();
//  //tp->saveGraphToFile( "graph.gv" );
//  //generatePngImage( "graph.gv" );
//}

BOOST_AUTO_TEST_CASE( test_LGP_blocks_fol_3w )
{
  // tests that Dijkstra works well from root and from other nodes
  auto tp = std::make_shared< tp::GraphSearchPlanner >();
  tp->setFol( "data/LGP-blocks-fol.g" );

  //
  auto start = std::chrono::high_resolution_clock::now();
  //

  tp->buildGraph();

  //
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  //

  std::cout << "graph build time (s):" << ms / 1000.0 << std::endl;

  //tp->saveGraphToFile( "graph.gv" );
  //generatePngImage( "graph.gv" );

//  std::unordered_set< std::string > s1;
//  std::unordered_set< std::string > s2;

//  std::set< std::string > inter;
//  std::set_intersection( s1.begin(), s1.end(),
//                         s2.begin(), s2.end(),
//                         std::inserter( inter, inter.begin() ) );

}

BOOST_AUTO_TEST_CASE( test_LGP_blocks_hard_fol_3w )
{
  // tests that Dijkstra works well from root and from other nodes
  auto tp = std::make_shared< tp::GraphSearchPlanner >();
  tp->setFol( "data/LGP-blocks-hard-fol.g" );

  //
  auto start = std::chrono::high_resolution_clock::now();
  //

  tp->buildGraph();

  //
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  //

  std::cout << "graph build time (s):" << ms / 1000.0 << std::endl;

  //tp->saveGraphToFile( "graph.gv" );
  //generatePngImage( "graph.gv" );

//  std::unordered_set< std::string > s1;
//  std::unordered_set< std::string > s2;

//  std::set< std::string > inter;
//  std::set_intersection( s1.begin(), s1.end(),
//                         s2.begin(), s2.end(),
//                         std::inserter( inter, inter.begin() ) );

}
