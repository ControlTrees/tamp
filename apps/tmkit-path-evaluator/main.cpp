#include <functional>
#include <list>

#include <chrono>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <KOMO/komo.h>
#include <Kin/taskMaps.h>

#include <path_evaluator.h>

#include <math_utility.h>

#include <amino.h>
#include <tmplan.h>

//===========================================================================

std::pair< double, double > evaluate( rai::Array< rai::KinematicWorld > & kinFrames, double tau );

void baxter()
{
  const std::string kinFile = "LGP-blocks-kin-1w-unified-4-blocks.g";
  std::list< std::string > fileNames;
  std::list< std::pair< double, double > > costs; // length + acc cost

  for( uint i = 1; i <= 100; ++i )
  {
    fileNames.push_back( "tm-3-blocks/baxter-sussman-3-blocks-" + std::to_string(i) + ".tmp" );
  }

  //fileNames.push_back( "baxter-sussman-6.tmp" );

  //
  for( const auto & file : fileNames )
  {
    ifstream f( file );
    if( f )
    {
      // parsing
      uint nActions = 0;
      std::vector< std::vector< std::pair< std::string, double > > > frames;
      std::vector< std::string> header;

      for( std::string line; getline( f, line ); )
      {
        for( auto & c : line )
        {
          if( c == '\t' )
            c = ' ';
        }
        using tokenizer = boost::tokenizer<boost::char_separator<char> >;
        boost::char_separator<char> sep( " " );
        tokenizer tokens( line, sep );
        const auto token = tokens.begin();

        if( *token == "m" )
        {
          nActions++;
          header.clear();
          for( auto token = ++tokens.begin(); token != tokens.end(); ++token )
          {
            header.push_back( *token );
          }
        }
        if( *token == "p" )
        {
          std::vector< std::pair< std::string, double > > frame;

          uint i = 0;
          for( auto token = ++tokens.begin(); token != tokens.end(); ++token, ++i )
          {
            frame.push_back( std::make_pair( header[ i ], std::stod( *token ) ) );
          }

          frames.push_back( frame );
        }
      }

      // convertion to KOMO
      rai::Array< rai::KinematicWorld > kinFrames;

      rai::KinematicWorld kin;
      kin.init( kinFile.c_str() );

      for( auto i = 0; i < frames.size(); ++i )
      {
        auto frame = frames[ i ];

        arr q_(frame.size());
        StringA names(frame.size());

        for( auto i = 0; i < frame.size(); ++i )
        {
          auto jointName = frame[i].first;
          auto q = frame[i].second;

          q_(i) = q;

          names(i) = jointName.c_str();
        }

        kin.setJointState( q_, names );

        //kin.watch();
        //rai::wait( 0.2, true );

        kinFrames.push_back( kin );
      }

      const double tau = double( nActions ) / frames.size() * 10.0;

      // evaluation
      auto eval = mp::evaluate( kinFrames, tau );

      auto length = eval.first;
      auto acc_cost = eval.second;

      costs.push_back( eval );

      std::cout << "length:" << length << " acc cost:" << acc_cost << std::endl;
    }
  }

  // post-process costs
  namespace ba = boost::accumulators;
  boost::accumulators::accumulator_set<double, ba::features< ba::tag::variance, ba::tag::mean, ba::tag::min, ba::tag::max > > acc_length;
  boost::accumulators::accumulator_set<double, ba::features< ba::tag::variance, ba::tag::mean, ba::tag::min, ba::tag::max > > acc_acc_cost;

  for( auto c : costs )
  {
    acc_length( c.first );
    acc_acc_cost( c.second );
  }

  std::cout << "LENGTH: [" <<  ba::min( acc_length ) << " " << ba::max( acc_length ) << "] mean:" << ba::mean( acc_length ) << " std_dev:" << sqrt( ba::variance( acc_length ) ) << std::endl;
  std::cout << "ACC COSTS: [" << ba::min( acc_acc_cost ) << " " << ba::max( acc_acc_cost ) << "] mean:" << ba::mean( acc_acc_cost ) << " std_dev:" << sqrt( ba::variance( acc_acc_cost ) ) << std::endl;
}


//===========================================================================

int main(int argc,char **argv)
{
  rai::initCmdLine(argc,argv);

  baxter();

  return 0;
}
