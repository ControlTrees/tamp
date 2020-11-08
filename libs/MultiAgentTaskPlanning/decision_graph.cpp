#include <decision_graph.h>

#include <set>
#include <queue>
#include <algorithm>

#include <decision_graph_printer.h>

namespace matp
{

bool sameState ( const NodeData & a, const NodeData & b )
{
  return a.hash() == b.hash();
}

std::ostream& operator<<(std::ostream& stream, NodeData const& data)
{
  stream << "states:" << std::endl;
  for( auto s : data.states )
  {
    stream << s << std::endl;
  }

  stream << "belief state:" << std::endl;
  for( auto p : data.beliefState )
  {
    stream << p << std::endl;
  }

  return stream;
}

std::vector < double > normalizeBs( const std::vector < double > & bs )
{
  std::vector < double > newBs = bs;

  auto sumfunc = []( const std::vector < double > bs ) -> double
  {
    double sum = 0;
    for( auto p : bs )
    {
      sum += p;
    }
    return sum;
  };

  const auto sum = sumfunc( bs );

  for( auto w = 0; w < bs.size(); ++w )
  {
    newBs[ w ] = bs[ w ] / sum;
  }

  CHECK( fabs( sumfunc( newBs ) - 1.0 ) < 0.00001, "" );

  return newBs;
}

void DecisionGraph::reset()
{
  //root_.reset();
  edges_.clear();
  nodes_.clear();
  hash_to_id_.clear();
  terminalNodes_.clear();
}

// copy
DecisionGraph::DecisionGraph( const DecisionGraph & graph ) // copy ctor
{
  copy( graph );
}

DecisionGraph& DecisionGraph::operator= ( const DecisionGraph & graph ) // assignment operator
{
  copy( graph );

  return *this;
}

// DecisionGraph
DecisionGraph::DecisionGraph( const LogicEngine & engine, const std::vector< std::string > & startStates, const std::vector< double > & egoBeliefState )
  : engine_( engine )
  , root_( GraphNode< NodeData >::root( NodeData( sortFacts( startStates ), egoBeliefState, false, 0, NodeData::NodeType::ACTION ) ) )
{
  // sort states before creating root
  std::vector< std::string > sortedStartState = startStates;

  for( auto & s : sortedStartState )
  {
    s = concatenateFacts( getFilteredFacts( s ) );
  }

  root_ = GraphNode< NodeData >::root( NodeData( sortedStartState, egoBeliefState, false, 0, NodeData::NodeType::ACTION ) );
  hash_to_id_[ root_->data().hash() ].push_back( 0 );

  nodes_.push_back( root_ );
  edges_.push_back( EdgeDataType() ); // dummy edge coming to root
}

void DecisionGraph::build( int maxSteps, bool graph )
{
  std::queue< GraphNode< NodeData >::ptr > queue;
  queue.push( root_ );
  isGraph_ = graph;

  uint step = 0;
  while( ! queue.empty() )
  {
    auto node = queue.front();
    queue.pop();

    step = node->depth() / 2 / engine_.agentNumber();

    if( step < maxSteps )
    {
      auto queueExtension = expand( node );

      while( ! queueExtension.empty() )
      {
        queue.push( std::move( queueExtension.front() ) );
        queueExtension.pop();
      }
    }
    else
    {
      break;
    }
  }
}

std::queue< GraphNode< NodeData >::ptr > DecisionGraph::expand( const GraphNode< NodeData >::ptr & node )
{
  const auto& bs     = node->data().beliefState;
  const auto& states = node->data().states;

  std::queue< GraphNode< NodeData >::ptr > nextQueue;

  nextQueue.push( node );

  // for each agent
  for( uint agentId = 0; agentId < engine_.agentNumber(); ++agentId )
  {
    auto queue = nextQueue; // copy
    nextQueue = std::queue< GraphNode< NodeData >::ptr >();

    while( ! queue.empty() )
    {
      const auto& node = queue.front();
      queue.pop();

      // for each agent
      const auto& actions = getCommonPossibleActions( node, agentId );

      for( const auto& action : actions )
      {
        // node after action, will receive one or several observations
        auto child = node->makeChild( GraphNodeDataType( states, bs, false, agentId, NodeData::NodeType::OBSERVATION ) );
        nodes_.push_back( child );
        edges_.push_back( { { node->id(), std::make_pair( 1.0, action ) } } );
        CHECK_EQ( edges_.size() - 1, child->id(), "corruption in edge data structure" );

        // for each outcome
        auto outcomes = getPossibleOutcomes( node, action );
        for( const auto& outcome : outcomes )
        {
          CHECK( node->data().agentId == agentId, "Corruption in the queue!" );
          auto nextAgentId = ( agentId + 1 ) % engine_.agentNumber();

          // outcome.p
          // node after observation, will have to choose between several actions
          auto p    = std::get<0>(outcome);
          auto data = std::get<1>(outcome);
          auto observation = std::get<2>(outcome);
          const auto childChildData = GraphNodeDataType( data.states, data.beliefState, data.terminal, nextAgentId, NodeData::NodeType::ACTION );

          bool nodeNeedsToBeCreated = true;
          if( isGraph_ )
          {
            if( hash_to_id_.count( childChildData.hash() ) )
            {
              const auto& childChildId = hash_to_id_[ childChildData.hash() ].front();
              const auto& childChild = nodes_[ childChildId ];

              child->addExistingChild( childChild.lock() );
              edges_[ childChildId ][ child->id() ] = std::make_pair( p, observation );

              nodeNeedsToBeCreated = false;
            }
          }
          // tree case
          if( nodeNeedsToBeCreated )
          {
            const auto childChild = child->makeChild( childChildData );

            hash_to_id_[ childChild->data().hash() ].push_back( childChild->id() );
            nodes_.push_back( childChild );
            edges_.push_back( { { child->id(), std::make_pair( p, observation ) } } );
            CHECK_EQ( edges_.size() - 1, childChild->id(), "corruption in edge data structure" );

            if( data.terminal )
            {
              terminalNodes_.push_back( childChild );
            }
            else
            {
              nextQueue.push( childChild );
            }
          }
        }
      }
    }
  }

  return nextQueue;
}

void DecisionGraph::_addNode( const std::weak_ptr< GraphNodeType > & _node )
{
  nodes_.push_back( _node );

  auto node = _node.lock();
  if( node->data().nodeType ==  DecisionGraph::GraphNodeDataType::NodeType::ACTION )
  {
    hash_to_id_[ node->data().hash() ].push_back( node->id() );

    if( node->data().terminal )
    {
      terminalNodes_.push_back( node );
    }
  }
}

void DecisionGraph::_addEdge( uint child, uint parent, double p, const std::string & artifact )
{
  if( child == edges_.size() )
  {
    edges_.push_back( { { parent, std::make_pair( p, artifact ) } } );
    CHECK_EQ( edges_.size()-1, child, "edges should be added in the right order!!" );
  }
  else
  {
    edges_[ child ][ parent ] = std::make_pair( p, artifact );
  }
}

void DecisionGraph::removeNode( const std::weak_ptr< GraphNodeType > & _node )
{
  auto node = _node.lock();

  // check in terminals
  for( auto tIt = terminalNodes_.begin(); tIt != terminalNodes_.end(); ++tIt )
  {
    if( tIt->lock()->id() == node->id() )
    {
      terminalNodes_.erase( tIt );
    }
  }

  // check in nodes
  for( auto tIt = nodes_.begin(); tIt != nodes_.end(); ++tIt )
  {
    if( tIt->lock()->id() == node->id() )
    {
      nodes_.erase( tIt );
    }
  }

  // inform parents
  for( auto _p : node->parents() )
  {
    auto p = _p.lock();
    if( p )
    {
      p->removeChild( node );
      nodes_[ node->id() ].reset();
    }
  }
}

void DecisionGraph::purgeNodes( const std::vector< bool > & toKeep ) // remove nodes from nodes_ and terminalNodes_ that are not valid anymore
{
  auto to_keep_lambda = [&] ( const std::weak_ptr< GraphNodeType > & node ) -> bool
  {
    return ! toKeep[ node.lock()->id() ];
  };
  nodes_.erase( std::remove_if(nodes_.begin(), nodes_.end(), to_keep_lambda), nodes_.end() );
  terminalNodes_.erase( std::remove_if(terminalNodes_.begin(), terminalNodes_.end(), to_keep_lambda), terminalNodes_.end() );
}

void DecisionGraph::saveGraphToFile( const std::string & filename ) const
{
  if( ! root_ )
  {
    return;
  }

  std::ofstream file;
  file.open( filename );

  GraphPrinter printer( file );
  printer.print( *this );

  file.close();

  // png
  std::string nameCopy( filename );
  const std::string ext( ".gv" );
  std::string newName = nameCopy.replace( nameCopy.find( ext ), ext.length(), ".png" );

  std::stringstream ss;
  ss << "dot"   << " ";
  ss << "-Tpng" << " ";
  ss << "-o"    << " ";
  ss << newName << " ";
  ss << filename;

  system( ss.str().c_str() );
}

std::vector< std::string > DecisionGraph::getCommonPossibleActions( const GraphNode< NodeData >::ptr & node, uint agentId ) const
{
  LogicEngine & engine = engine_; // copy to be const
  auto bs     = node->data().beliefState;
  auto states = node->data().states;

  /// get possible actions
  std::vector< std::string > possibleActions;
  for( auto w = 0; w < bs.size(); ++w )
  {
    if( bs[ w ] > 0 )
    {
      auto startState = states[ w ];
      engine.setState( startState );

      auto newActions = engine.getPossibleActions( agentId );

      if( possibleActions.empty() )
      {
        possibleActions = newActions;
      }
      else
      {
        std::vector< std::string > newPossibleActions;

        // each action in the new possible actions is has to be previously in the previous possible actions
        std::set_intersection( possibleActions.begin(), possibleActions.end(),
                               newActions.begin(), newActions.end(),
                               std::back_inserter( newPossibleActions ) );

        possibleActions = newPossibleActions;
      }
    }
  }

  return possibleActions;
}

std::vector< std::tuple< double, NodeData, std::string > > DecisionGraph::getPossibleOutcomes( const GraphNode< NodeData >::ptr & node, const std::string & action ) const
{
  //std::cout << "DecisionGraph::getPossibleOutcomes of " << node->id() << std::endl; // tmp camille

  std::vector< std::tuple< double, NodeData, std::string > > outcomes;

  LogicEngine & engine = engine_; // copy to be const
  auto bs     = node->data().beliefState;
  auto states = node->data().states;

  //        observable facts                                    world state
  std::map< std::set< std::string >, std::vector< std::pair< uint, std::string > > > observableStatesToStates;
  std::map< std::set< std::string >, bool > terminalOutcome;
  std::set< std::string > factIntersection;

  for( auto w = 0; w < bs.size(); ++w )
  {
    if( bs[ w ] > 0 )
    {
      auto startState = states[ w ];
      engine.setState( startState );

      engine.transition( action );

      const auto _result          = engine.getState();
      const auto facts            = getFilteredFacts( _result );// without komo and action tags
      const auto result           = concatenateFacts( facts );
      const auto observableFacts  = getObservableFacts( facts );
      const auto terminal         = engine.isTerminal();

      //std::cout << result << std::endl; // tmp camille

      std::set< std::string > newIntersection;

      if( factIntersection.empty() )
      {
        newIntersection = facts;
      }
      else
      {
        std::set_intersection( facts.begin(), facts.end(), factIntersection.begin(), factIntersection.end(),
                               std::inserter( newIntersection, newIntersection.begin() ) );
      }

      // store results
      factIntersection = newIntersection;
      observableStatesToStates[ observableFacts ].push_back( std::make_pair( w, result ) );
      terminalOutcome         [ observableFacts ] = terminal;
    }
  }

  //
  for( auto observableResultPair : observableStatesToStates )
  {
    std::vector< std::string > states( bs.size(), "" );
    std::vector< double > newBs( bs.size(), 0 );

    const auto & worldToOutcomes = observableResultPair.second;
    auto observationFacts = getEmergingFacts( factIntersection, observableResultPair.first );
    auto observation      = concatenateFacts( observationFacts );
    auto terminal         = terminalOutcome[ observableResultPair.first ];

    double p = 0;
    for( const auto & worldOutcome :  worldToOutcomes )
    {
      auto w = worldOutcome.first;
      auto state = worldOutcome.second;

      p += bs[ w ];
      states[ w ] = state;
      newBs[ w ] = bs[ w ];
    }

    newBs = normalizeBs( newBs );

    outcomes.push_back( std::make_tuple( p, NodeData( states, newBs, terminal, node->data().agentId, NodeData::NodeType::OBSERVATION ), observation ) );
  }

  return outcomes;
}

void DecisionGraph::copy( const DecisionGraph & graph )
{
  reset();

  if( graph.root() )
  {
    engine_ = graph.engine_;
    isGraph_ = graph.isGraph_;
    edges_ = graph.edges_;

    auto rootData = graph.root()->data();

    root_ = GraphNodeType::root( rootData );
    hash_to_id_[ root_->data().hash() ].push_back( 0 );
    nodes_.push_back( root_ );

    std::queue< std::pair < GraphNodeType::ptr, GraphNodeType::ptr > > Q;

    Q.push( std::make_pair( graph.root(), root_ ) ); // original - copy

    while( ! Q.empty() )
    {
      auto u = Q.front();
      Q.pop();

      auto uOriginal = u.first;
      auto uCopy     = u.second;

      CHECK( uCopy->data().nodeType == GraphNodeDataType::NodeType::ACTION, "wrong node type" );

      for( auto vOriginal : uOriginal->children() )
      {        
        auto vCopy = uCopy->makeChild( vOriginal->data() );
        vCopy->setId( vOriginal->id() );
        nodes_.push_back( vCopy );

        //std::cout << "copy " << vCopy->id() << std::endl;

        CHECK( ! vCopy->data().terminal, "termination can appear only after the observation!" );
        CHECK( vCopy->data().nodeType == GraphNodeDataType::NodeType::OBSERVATION, "wrong node type" );

        for( auto wOriginal : vOriginal->children() )
        {
          CHECK( wOriginal->data().nodeType == GraphNodeDataType::NodeType::ACTION, "wrong node type" );

          if( isGraph_ && hash_to_id_.count( wOriginal->data().hash() ) != 0 )
          {
            //std::cout << "rewire to " << wOriginal->id() << std::endl;

            CHECK( hash_to_id_.at( wOriginal->data().hash() ).size() == 1, "datastructure corruption" );
            CHECK( hash_to_id_.at( wOriginal->data().hash() ).front() == wOriginal->id(), "datastructure corruption" );
            auto wCopy = nodes_[ wOriginal->id() ].lock();
            vCopy->addExistingChild( wCopy );
          }
          else
          {
            auto wCopy = vCopy->makeChild( wOriginal->data() );
            wCopy->setId( wOriginal->id() );
            nodes_.push_back( wCopy );
            hash_to_id_[ wCopy->data().hash() ].push_back( wCopy->id() );

            //std::cout << "copy " << wCopy->id() << std::endl;

            if( wCopy->data().terminal )
            {
              terminalNodes_.push_back( wCopy );
            }
            else
            {
              Q.push( std::make_pair( wOriginal, wCopy ) );
            }
          }
        }
      }
    }
  }
}

} // namespace matp
