#include <graph_planner.h>

#include <algorithm>    // std::random_shuffle
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

static double m_inf() { return std::numeric_limits< double >::lowest(); }

namespace matp
{

std::vector< double > ValueIterationAlgorithm::process( const DecisionGraph & graph, Rewards & rewards )
{
  auto fromToIndex = [&graph] ( uint from, uint to )->uint
  {
    return from * graph.size() + to;
  };

  std::cout << "valueIteration.. start" << std::endl;

  using NodeTypePtr = std::shared_ptr< DecisionGraph::GraphNodeType >;

  double alpha = 0.5;
  const double initValue = -10;

  std::vector< double > values = std::vector< double >( graph.size(), initValue ); // magic value!! distance from root to vertex[i]

  auto comp = [ & ]( const NodeTypePtr & a, const NodeTypePtr & b ) -> bool
  {
    return values[ a->id() ] < values[ b->id() ];
  };

  auto diff_func = []( double a, double b ) -> double
  {
    const double gentle_m_inf = -10e8;
    if( a < gentle_m_inf && b < gentle_m_inf )
    {
      return 0;
    }
    else if( a >= gentle_m_inf && b >= gentle_m_inf )
    {
      return fabs( a -b );
    }
    else
    {
      return std::numeric_limits< double >::infinity();
    }
  };

  // go from leafs to root
  const auto& nodes = graph.nodes();
  const auto& terminals = graph.terminalNodes();
  auto edges = graph.edges();

  for( const auto& weakV : terminals )
  {
    const auto& v = weakV.lock();

    values[ v->id() ] = 0; // all rewards negative
  }

  // expected reward up to terminal nodes
  // add terminal nodes to Q
  uint totalUpdates = 0;
  constexpr double eps = 10e-5;
  bool stable = false;
  for( auto i = 0; ! stable && i < 1000; ++i )
  {
    double maxDiff = 0;

    auto itNodes = nodes;
    std::random_shuffle ( itNodes.begin(), itNodes.end() );

    for( const auto& weakU : itNodes )
    {
      const auto& u = weakU.lock();

      if( u->data().nodeType == NodeData::NodeType::ACTION )
      {
        if( u->data().agentId == 0 )
        {
          if( ! u->data().terminal )
          {
            double newTargetValue = m_inf(); // if no children and not terminal, it means that it is infeasible hence m_inf

            // max operation, choose the best child
            for( auto v : u->children() )
            {
              const auto r = rewards.get( fromToIndex( u->id(), v->id() ) );

              if( values[ v->id() ] + r > newTargetValue )
              {
                newTargetValue = values[ v->id() ] + r;
              }
            }

            if( newTargetValue == m_inf() ) // there were no children so unfeasible
            {
              const auto diff = diff_func( values[ u->id() ], m_inf() );
              maxDiff = std::max( maxDiff, diff );

              //std::cout << "A update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << m_inf() << std::endl;
              //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

              values[ u->id() ] = m_inf();
            }
            else
            {
              const auto newValue = values[ u->id() ] * ( 1 - alpha ) + alpha * newTargetValue;
              const auto diff = diff_func( values[ u->id() ], newValue );
              maxDiff = std::max( maxDiff, diff );

              //std::cout << "B update " << u->id() << " old value:" << values[ u->id() ] << " new value:" << newValue << std::endl;
              //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

              values[ u->id() ] = newValue;
            }
          }
        }
        else // other agent
        {
          double newTargetValue = 0;

          uint n = u->children().size();
          for( auto v : u->children() )
          {
            newTargetValue += 1.0 / n * values[ v->id() ] ; // average
          }

          const auto newValue = values[ u->id() ] * ( 1 - alpha ) + alpha * newTargetValue;
          const auto diff = diff_func( values[ u->id() ], newValue );
          maxDiff = std::max( maxDiff, diff );

          //std::cout << "C update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << newValue << std::endl;
          //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

          values[ u->id() ] = newValue;
        }
      }
      else if( u->data().nodeType == NodeData::NodeType::OBSERVATION )
      {
        double newTargetValue = 0;
        double sumP = 0;
        for( auto v : u->children() )
        {
          CHECK_EQ( edges[ v->id() ].count( u->id() ), 1, "corruption in edge data structure" );
          const double p = edges[ v->id() ][ u->id() ].first;
          sumP+=p;
          newTargetValue += p * values[ v->id() ] ;
        }

        CHECK( fabs(sumP-1.0) < 0.001, "wrong belief state!" );

        const auto newValue = values[ u->id() ] * ( 1 - alpha ) + alpha * newTargetValue;
        const auto diff = diff_func( values[ u->id() ], newValue );
        maxDiff = std::max( maxDiff, diff );

        //std::cout << "D update " << u->id() << " old value:" << values[ u->id() ] << " new value:" << newValue << std::endl;
        //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

        values[ u->id() ] = newValue;
      }
    }

    stable = maxDiff < eps;

    //std::cout << "it: " << i << " maxDiff: " << maxDiff << std::endl;
  }

  if(!stable)
  {
    std::cout << "warning: Value iteration finished without convergence!" << std::endl;
  }

  std::cout << "valueIteration.. end" << std::endl;

  return values;

  //  std::cout << "GraphPlanner::valueIteration.. start" << std::endl;

  //  using NodeTypePtr = std::shared_ptr< DecisionGraph::GraphNodeType >;

  //  double alpha = 0.5;
  //  const double initValue = 100 * maxDepth_ * r0_;

  //  values_ = std::vector< double >( graph_.size(), initValue ); // magic value!! distance from root to vertex[i]

  //  auto comp = [ & ]( const NodeTypePtr & a, const NodeTypePtr & b ) -> bool
  //  {
  //    return values_[ a->id() ] < values_[ b->id() ];
  //  };

  //  auto diff_func = []( double a, double b ) -> double
  //  {
  //    const double gentle_m_inf = -10e8;
  //    if( a < gentle_m_inf && b < gentle_m_inf )
  //    {
  //      return 0;
  //    }
  //    else if( a >= gentle_m_inf && b >= gentle_m_inf )
  //    {
  //      return fabs( a -b );
  //    }
  //    else
  //    {
  //      return std::numeric_limits< double >::infinity();
  //    }
  //  };

  //  // go from leafs to root
  //  const auto nodes = graph_.nodes();
  //  auto terminals = graph_.terminalNodes();
  //  for( auto weakV : terminals )
  //  {
  //    auto v = weakV.lock();

  //    values_[ v->id() ] = 0; // all rewards negative
  //  }

  //  // expected reward up to terminal nodes
  //  // add terminal nodes to Q
  //  uint totalUpdates = 0;
  //  constexpr double eps = 10e-4;
  //  bool stable = false;
  //  for( auto i = 0; i < 100; ++i )
  //  {
  //    double maxDiff = 0;

  //    std::vector< bool > updated( graph_.size(), false ); // magic value!! distance from root to vertex[i]
  //    std::queue< std::weak_ptr< DecisionGraph::GraphNodeType > > Q;
  //    for( auto weakV : terminals )
  //    {
  //      Q.push( weakV );
  //    }

  //    while( ! Q.empty() )
  //    {
  //      auto u = Q.front().lock();
  //      Q.pop();

  //      if( u->data().nodeType == NodeData::NodeType::ACTION )
  //      {
  //        if( u->data().agentId == 0 )
  //        {
  //          if( ! u->data().terminal )
  //          {
  //            double newTargetValue = m_inf(); // if no children and not terminal, it means that it is infeasible hence m_inf

  //            // max operation, choose the best child
  //            for( auto v : u->children() )
  //            {
  //              const auto r = rewards_[ v->id() ];

  //              if( values_[ v->id() ] + r > newTargetValue )
  //              {
  //                newTargetValue = values_[ v->id() ] + r;
  //              }
  //            }

  //            if( newTargetValue == m_inf() ) // there were no children so unfeasible
  //            {
  //              const auto diff = diff_func( values_[ u->id() ], m_inf() );
  //              maxDiff = std::max( maxDiff, diff );

  //              //std::cout << "A update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << m_inf() << std::endl;
  //              //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

  //              values_[ u->id() ] = m_inf();
  //            }
  //            else
  //            {
  //              const auto newValue = values_[ u->id() ] * ( 1 - alpha ) + alpha * newTargetValue;
  //              const auto diff = diff_func( values_[ u->id() ], newValue );
  //              maxDiff = std::max( maxDiff, diff );

  //              //std::cout << "B update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << newValue << std::endl;
  //              //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

  //              values_[ u->id() ] = newValue;
  //            }
  //          }
  //        }
  //        else // other agent
  //        {
  //          double newTargetValue = 0;

  //          uint n = u->children().size();
  //          for( auto v : u->children() )
  //          {
  //            newTargetValue += 1.0 / n * values_[ v->id() ] ; // average
  //          }

  //          const auto newValue = values_[ u->id() ] * ( 1 - alpha ) + alpha * newTargetValue;
  //          const auto diff = diff_func( values_[ u->id() ], newValue );
  //          maxDiff = std::max( maxDiff, diff );

  //          //std::cout << "C update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << newValue << std::endl;
  //          //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

  //          values_[ u->id() ] = newValue;
  //        }
  //      }
  //      else if( u->data().nodeType == NodeData::NodeType::OBSERVATION )
  //      {
  //        double newTargetValue = 0;

  //        for( auto v : u->children() )
  //        {
  //          newTargetValue += v->data().p * values_[ v->id() ] ;
  //        }

  //        const auto newValue = values_[ u->id() ] * ( 1 - alpha ) + alpha * newTargetValue;
  //        const auto diff = diff_func( values_[ u->id() ], newValue );
  //        maxDiff = std::max( maxDiff, diff );

  //        //std::cout << "D update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << newValue << std::endl;
  //        //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

  //        values_[ u->id() ] = newValue;
  //      }

  //      updated[ u->id() ] = true;

  //      //
  //      for( auto p : u->parents() )
  //      {
  //        if( ! updated[ p.lock()->id() ] )
  //        {
  //          Q.push( p );
  //        }
  //      }
  //    }

  //    stable = maxDiff < eps;

  //    std::cout << "it: " << i << " maxDiff: " << maxDiff << std::endl;
  //  }

  //  std::cout << "GraphPlanner::valueIteration.. end" << std::endl;

    ///////////////////////////////////////////////////////////////////////

  //  std::cout << "GraphPlanner::valueIteration.. start" << std::endl;

  //  using NodeTypePtr = std::shared_ptr< DecisionGraph::GraphNodeType >;

  //  double alpha = 0.5;
  //  const double initValue = 100 * maxDepth_ * r0_;

  //  values_ = std::vector< double >( graph_.size(), initValue ); // magic value!! distance from root to vertex[i]

  //  auto comp = [ & ]( const NodeTypePtr & a, const NodeTypePtr & b ) -> bool
  //  {
  //    return values_[ a->id() ] < values_[ b->id() ];
  //  };

  //  auto diff_func = []( double a, double b ) -> double
  //  {
  //    const double gentle_m_inf = -10e8;
  //    if( a < gentle_m_inf && b < gentle_m_inf )
  //    {
  //      return 0;
  //    }
  //    else if( a >= gentle_m_inf && b >= gentle_m_inf )
  //    {
  //      return fabs( a -b );
  //    }
  //    else
  //    {
  //      return std::numeric_limits< double >::infinity();
  //    }
  //  };

  //  // go from leafs to root
  //  const auto nodes = graph_.nodes();
  //  auto terminals = graph_.terminalNodes();
  //  for( auto weakV : terminals )
  //  {
  //    auto v = weakV.lock();

  //    values_[ v->id() ] = 0; // all rewards negative
  //  }

  //  // expected reward up to terminal nodes
  //  // add terminal nodes to Q
  //  uint totalUpdates = 0;
  //  constexpr double eps = 10e-4;
  //  bool stable = false;
  //  for( auto i = 0; ! stable && i < 1000; ++i )
  //  {
  //    double maxDiff = 0;

  //    auto itNodes = nodes;
  //    std::random_shuffle ( itNodes.begin(), itNodes.end() );

  //    for( auto weakU : itNodes )
  //    {
  //      auto u = weakU.lock();

  //      if( u->data().nodeType == NodeData::NodeType::ACTION )
  //      {
  //        if( u->data().agentId == 0 )
  //        {
  //          if( ! u->data().terminal )
  //          {
  //            double newTargetValue = m_inf(); // if no children and not terminal, it means that it is infeasible hence m_inf

  //            // max operation, choose the best child
  //            for( auto v : u->children() )
  //            {
  //              const auto r = rewards_[ v->id() ];

  //              if( values_[ v->id() ] + r > newTargetValue )
  //              {
  //                newTargetValue = values_[ v->id() ] + r;
  //              }
  //            }

  //            if( newTargetValue == m_inf() ) // there were no children so unfeasible
  //            {
  //              const auto diff = diff_func( values_[ u->id() ], m_inf() );
  //              maxDiff = std::max( maxDiff, diff );

  //              //std::cout << "A update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << m_inf() << std::endl;
  //              //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

  //              values_[ u->id() ] = m_inf();
  //            }
  //            else
  //            {
  //              const auto newValue = values_[ u->id() ] * ( 1 - alpha ) + alpha * newTargetValue;
  //              const auto diff = diff_func( values_[ u->id() ], newValue );
  //              maxDiff = std::max( maxDiff, diff );

  //              //std::cout << "B update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << newValue << std::endl;
  //              //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

  //              values_[ u->id() ] = newValue;
  //            }
  //          }
  //        }
  //        else // other agent
  //        {
  //          double newTargetValue = 0;

  //          uint n = u->children().size();
  //          for( auto v : u->children() )
  //          {
  //            newTargetValue += 1.0 / n * values_[ v->id() ] ; // average
  //          }

  //          const auto newValue = values_[ u->id() ] * ( 1 - alpha ) + alpha * newTargetValue;
  //          const auto diff = diff_func( values_[ u->id() ], newValue );
  //          maxDiff = std::max( maxDiff, diff );

  //          //std::cout << "C update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << newValue << std::endl;
  //          //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

  //          values_[ u->id() ] = newValue;
  //        }
  //      }
  //      else if( u->data().nodeType == NodeData::NodeType::OBSERVATION )
  //      {
  //        double newTargetValue = 0;

  //        for( auto v : u->children() )
  //        {
  //          newTargetValue += v->data().p * values_[ v->id() ] ;
  //        }

  //        const auto newValue = values_[ u->id() ] * ( 1 - alpha ) + alpha * newTargetValue;
  //        const auto diff = diff_func( values_[ u->id() ], newValue );
  //        maxDiff = std::max( maxDiff, diff );

  //        //std::cout << "D update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << newValue << std::endl;
  //        //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

  //        values_[ u->id() ] = newValue;
  //      }
  //    }

  //    stable = maxDiff < eps;

  //    std::cout << "it: " << i << " maxDiff: " << maxDiff << std::endl;
  //  }

  //  std::cout << "GraphPlanner::valueIteration.. end" << std::endl;
}

}
