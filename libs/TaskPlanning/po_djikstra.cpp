#include <po_djikstra.h>
#include <queue>

namespace tp
{

//---------Dijkstra-----------------//

Dijkstra::Dijkstra( const rai::Array< std::shared_ptr< FOL_World > > & folEngines )
  : folEngines_( folEngines )
{

}

Policy::ptr Dijkstra::solve( const POWeightedGraph::ptr & graph, const POGraphNode::ptr & from )
{
  // reset state
  values_.clear();
  bestFamily_.clear();
  parents_.clear();
  policy_.reset();
  PO2Policy_.clear();
  Policy2PO_.clear();
  //

  graph_ = graph;

  // determines value of each node
  dijkstra( graph_->terminals() );

  // fill bestFamily_ and parents_
  bool found = extractSolutionFrom( from );

  if( found )
  {
    // build policy data-structure
    buildPolicy( from );
  }

  CHECK( ( ! found && ! policy_ ) || ( found && policy_ ), "inconsistency!" );

  return policy_;
}

void Dijkstra::dijkstra( const std::list < POGraphNode::ptr > & terminals )
{
  std::cout << "GraphSearchPlanner::dijkstra.." << std::endl;

  values_ = std::vector< double >( graph_->size(), m_inf() ); // distance from root to vertex[i]

  auto comp = [ & ]( const POGraphNode::ptr & a, const POGraphNode::ptr & b ) -> bool
  {
    return values_[ a->id() ] < values_[ b->id() ];
  };

  bestFamily_ = std::vector< int >( graph_->size(), -1 );
  parents_    = std::vector< POGraphNode::ptr >( graph_->size() );

  std::vector< uint > nUpdates( graph_->size(), 0 );
  std::priority_queue< POGraphNode::ptr, std::vector< POGraphNode::ptr >, decltype( comp ) > Q( comp );

  // go from leafs to root
  for( auto v : terminals )
  {
    values_[ v->id() ] = 0; // all rewards negative
  }

  // expected reward up to terminal nodes
  // add terminal nodes to Q
  uint totalUpdates = 0;
  bool stable = false;
  for( auto i = 0; ! stable && i < 1000; ++i )
  {
    stable = true;
    // go from leafs to root
    for( auto v : terminals )
    {
      Q.push( v );
    }

    // algorithm
    while( ! Q.empty() )
    {
      auto u = Q.top();
      Q.pop();

      for( auto parent : u->parents() )
      {
        if( graph_->edgePossible( parent->id(), u->id() ) )
        {
          const auto r = graph_->reward( parent->id(), u->id() );

          double one = 0;

          one += u->q();
          auto alternativeParentValue = u->q() * ( values_[ u->id() ] + r ); // p->(u,v)

          for( auto v : u->andSiblings() )
          {
            if( ! graph_->edgePossible( parent->id(), v->id() ) )
            {
              alternativeParentValue = m_inf(); one = 1.0; break;
            }

            const auto r = graph_->reward( parent->id(), v->id() );

            one += v->q();
            alternativeParentValue += v->q() * ( values_[ v->id() ] + r );
          }

          CHECK( fabs( 1.0 - one ) < eps(), "corruption in probability computation!!" );

          if( alternativeParentValue > values_[ parent->id() ] )
          {
//            if( nUpdates[ parent->id() ] == 0 )
//            {
//              values_[ parent->id() ] = alternativeParentValue;
//            }
//            else
//            {
//              values_[ parent->id() ] = values_[ parent->id() ]alternativeParentValue;
//            }
            values_[ parent->id() ] = alternativeParentValue;
            stable = false;
            nUpdates[ parent->id() ]++;
            totalUpdates++;

            Q.push( parent );
          }
        }
      }
    }

    }

  std::cout << "GraphSearchPlanner::dijkstra.. end" << std::endl;

//  // go from leafs to root
//  for( auto v : terminals )
//  {
//    values_[ v->id() ] = 0; // all rewards negative
//    Q.push( v );
//  }

//  // algorithm
//  while( ! Q.empty() )
//  {
//    auto u = Q.top();
//    Q.pop();

//    for( auto parent : u->parents() )
//    {
//      if( graph_->edgePossible( parent->id(), u->id() ) )
//      {
//        const auto r = graph_->reward( parent->id(), u->id() );

//        double one = 0;

//        one += u->q();
//        auto alternativeParentValue = u->q() * ( values_[ u->id() ] + r ); // p->(u,v)

//        for( auto v : u->andSiblings() )
//        {
//          if( ! graph_->edgePossible( parent->id(), v->id() ) )
//          {
//            alternativeParentValue = m_inf(); one = 1.0; break;
//          }

//          const auto r = graph_->reward( parent->id(), v->id() );

//          one += v->q();
//          alternativeParentValue += v->q() * ( values_[ v->id() ] + r );
//        }

//        CHECK( fabs( 1.0 - one ) < eps(), "corruption in probability computation!!" );

//        if( alternativeParentValue > values_[ parent->id() ] )
//        {
//          values_[ parent->id() ] = alternativeParentValue;
//          Q.push( parent );
//        }
//      }
//    }
//  }

//  }
}

bool Dijkstra::extractSolutionFrom( const POGraphNode::ptr & node )
{
  //std::cout << "extract solution from:" << node->id() << std::endl;
  double valueFromNode = values_[ node->id() ];

  if( valueFromNode <= m_inf() )
  {
    return false;
  }

  int bestFamily = -1; double bestValue = m_inf();
  for( auto i = 0; i < node->families().size(); ++i )
  {
    auto f = node->families()( i );

    double familyValue = 0;

    for( auto c : f )
    {
      double is_removed = ! graph_->edgePossible( node->id(), c->id() );
      if( ! is_removed )
      {
        familyValue += c->q() * values_[ c->id() ];
      }
      else
      {
        familyValue = m_inf();
      }
    }

    if( familyValue > bestValue )
    {
      bestFamily = i;
      bestValue = familyValue;
    }
  }

  bestFamily_[ node->id() ] = bestFamily;

  auto f = node->families()( bestFamily );

  for( auto c : f )
  {
    parents_[ c->id() ] = node;

    if( ! c->isTerminal() && node->id() != c->id() )
    {
      extractSolutionFrom( c );
    }
  }

  //
  CHECK( bestFamily_[ node->id() ] != -1 || node->isTerminal(), "since we extract the policy from this node, it should not be -1" );
  //

  return true;
}

bool Dijkstra::buildPolicy( const POGraphNode::ptr & from )
{
  // clear last correspondance
  PO2Policy_.clear();
  Policy2PO_.clear();

  if( values_[ from->id() ] == m_inf() )
  {
    // no solution has been found
    policy_.reset();
  }
  else
  {
    // convert to a policy object
    policy_ = std::make_shared< Policy >();
    policy_->init( from->N() );

    buildPolicyFrom( from, from );
  }

  checkValuesIntegrity( policy_ );
}

bool Dijkstra::buildPolicyFrom( const POGraphNode::ptr & node, const POGraphNode::ptr & start )
{
  PolicyNode::ptr policyNode = std::make_shared< PolicyNode >();

  if( node->id() == start->id() )
  {
    policyNode->setTime( 0 );
    policyNode->setState( node->folStates(), node->bs() );
    policyNode->setValue( values_[ node->id() ] );

    policy_->setRoot( policyNode );
    policy_->setValue( values_[ node->id() ] );
  }
  else
  {
    // set parent
    auto graphParent = parents_[ node->id() ];
    auto parent = PO2Policy_[ graphParent ];
    policyNode->setParent( parent );

    // get action graph ( have to be reconstructed with the right action! since it is a graph!!)
    uint a = node->getLeadingActionFrom( graphParent );
    policyNode->setLastReward( graph_->reward( parent->id(), node->id() ) );

    auto parentStates = graphParent->folStates(); // start states
    rai::Array< std::shared_ptr<Graph> > resultStates( parentStates.d0 );
    for( auto w = 0; w < node->N(); ++w )
    {
      if( node->bs()( w ) > eps() )
      {
        auto fol = folEngines_( w );
        auto startState = parentStates( w );
        fol->setState( startState.get() );
        auto action = fol->get_actions()[ a ];
        fol->transition( action );
        auto resultState = fol->createStateCopy();
        resultStates( w ).reset( resultState );
      }
    }
    policyNode->setState( resultStates, node->bs() );

    // add child to parent
    policyNode->setTime( parent->time() + 1 );
    policyNode->setValue( values_[ node->id() ] );
    policyNode->setStatus( PolicyNode::SKELETON );

    parent->addChild( policyNode );
    parent->setNextAction( node->getLeadingActionFromStr( graphParent ) );

    if( node->isTerminal() )
    {
      policy_->addLeaf( policyNode );
    }
  }

  // set node data
  policyNode->setId( node->id() );
  policyNode->setDifferentiatingFact( node->differentiatingFacts() );
  policyNode->setP( node->p() );
  policyNode->setQ( node->q() ); //

  // save correspondance
  PO2Policy_[ node ] = policyNode;
  Policy2PO_[ policyNode ] = node;

  if( ! node->isTerminal() )
  {
    auto bestFamily = node->families()( bestFamily_[ node->id() ] );
    for( auto c : bestFamily )
    {
      buildPolicyFrom( c, start );
    }
  }
}

void Dijkstra::checkValuesIntegrity( const Policy::ptr & policy )
{
  std::queue< PolicyNode::ptr > Q;

  Q.push( policy_->root() );

  while( ! Q.empty() )
  {
    auto node = Q.front();
    Q.pop();

    auto value = node->value();

    if( node->children().size() == 0 )
    {
      CHECK( value == 0, "terminal node without null value!" );
    }
    else
    {
      double recalculatedValue = 0;
      double one = 0;

      for( auto c : node->children() )
      {
        auto r = c->lastReward();
        //auto p = c->p();
        auto q = c->q();
        auto v = c->value();

        recalculatedValue += q * ( r + v );
        one += q;

        Q.push( c );
      }

      if( fabs( value - recalculatedValue ) > 0.0001 )
      {
        std::cout << "warning!!! policy values don't seem to be consistent!!" << std::endl;
      }
      //CHECK( fabs( value - recalculatedValue ) < 0.001, "wrong policy values" );
      CHECK( fabs( one - 1.0 ) < eps(), "wrong policy probabilities" );
    }
  }
}

}

