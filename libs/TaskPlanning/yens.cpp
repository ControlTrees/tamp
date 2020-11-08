#include <yens.h>

namespace tp {

//---------Yens--------------------//

Yens::Yens( const rai::Array< std::shared_ptr<FOL_World> > & folEngines )
  : folEngines_( folEngines )
  , dijkstra_  ( folEngines )
{

}

std::list< Policy::ptr > Yens::solve( const POWeightedGraph::ptr & graph, const uint K )
{
  graph_ = graph;

  // sort policies
  const auto policyComp = []( const Policy::ptr & a, const Policy::ptr & b )
  {
    return a->value() > b->value();
  };
  //

  std::list< Policy::ptr > policies;    // A
  std::list< Policy::ptr > altPolicies; // B

  auto policy_0 = dijkstra_.solve( graph, graph->root() );
  policies.push_back( policy_0 );

  // create the mask of edges to remove
  auto graphCopy = graph->clone();

  for( auto k = 1; k <= K; ++k )
  {
    //std::cout << "k=" << k << std::endl;

    auto lastPolicy = policies.back();

    // serialize the solution
    auto s_lastPolicy = serialize( lastPolicy );

    for( auto i = 0; i < s_lastPolicy.size(); ++i )
    {
      std::cout << "i=" << i << std::endl;

      auto spurNodeIt = s_lastPolicy.begin();
      std::advance( spurNodeIt, i );
      auto spurNode   = *spurNodeIt;
      auto rootPath = std::list< PolicyNode::ptr >( std::begin( s_lastPolicy ), spurNodeIt );

      //std::cout << "spurnode=" << spurNode->id() << std::endl;

      if( ! spurNode->children().empty() )
      {
        for( auto previousPolicy : policies )
        {
          auto s_previousPolicy = serialize( previousPolicy );
          auto ithNodeIt = s_previousPolicy.begin();
          std::advance( ithNodeIt, i );
          auto previousRootPath = std::list< PolicyNode::ptr >( std::begin( s_previousPolicy ), ithNodeIt );

          if( equivalent( rootPath, previousRootPath ) )
          {
            // Remove the choice that has already been done by the previous policies at the spur node
            // Remove the links that are part of the previous shortest paths which share the same root path.
            auto fromNode = (*ithNodeIt);
            //auto to   = (*(++ithNodeIt))->id();
            for( auto c : fromNode->children() )
            {
              auto from = fromNode->id();
              auto to   = c->id();

              std::cout << "delete edge:" << from << "-" << to << std::endl;

              graphCopy->removeEdge( from, to );
            }
          }

          // Remove the the root path
          for( auto n : rootPath )
          {
            // Remove n
            if( n->id() != spurNode->id() )
            {
              // remove node and sibling
              auto parent = n->parent();
              if( ! parent )
              {
                graphCopy->removeNode( n->id() );
              }
              else
              {
                for( auto n : parent->children() )
                {
                  graphCopy->removeNode( n->id() );
                }
              }
            }
          }
        }

        auto spurPolicy = dijkstra_.solve( graphCopy, graphCopy->getNode( spurNode->id() ) );

        if( spurPolicy )
        {
          //std::cout << "alternative policy found!" << std::endl;

          auto altPolicy = fuse( lastPolicy, spurPolicy );

          altPolicies.push_back( altPolicy );
        }
      }

      // reset
      graphCopy->reset();
    }

    // add the best alternative policy
    //std::cout << altPolicies.size() << " alternative policies for k=" << k << std::endl;
    altPolicies.sort( policyComp );

    if( ! altPolicies.empty() )
    {
      policies.push_back( altPolicies.front() );
      altPolicies.pop_front();
    }
  }

  return policies;
}

}
