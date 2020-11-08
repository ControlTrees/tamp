/*  ------------------------------------------------------------------
    Copyright 2016 Camille Phiquepal
    email: camille.phiquepal@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "po_graph_node.h"

#include <unordered_map>

#include <list>

#include <chrono>

#include <boost/algorithm/string/replace.hpp>

#include <Core/util.h>

#define DEBUG(x) //x
#define DEL_INFEASIBLE(x) x


//=====================free functions======================

namespace tp
{
double m_inf() { return std::numeric_limits< double >::lowest(); }
double eps() { return 1000 * std::numeric_limits< double >::epsilon(); }

static std::string toStdString( Node * node )
{
  std::stringstream ss;
  ss << * node;
  return ss.str();
}

static SymbolicState getStateStr( Graph * state )
{
  SymbolicState s;

  std::stringstream ss;
  state->write( ss," ","{}" );

  s.state = ss.str();

  for( auto node : * state )
  {
    //std::cout << * node << std::endl;

    std::stringstream ss;
    ss << * node;
    auto fact = ss.str();

    if( fact.find( "decision" ) == std::string::npos
        &&
        fact.find( "komo" ) == std::string::npos
        )
      s.facts.insert( fact );
  }

  s.factsHash_ = StringSetHash()( s.facts );

  return s;
}

static std::set< std::string > getObservableStateStr( Graph * state )
{
  // look for potential partial observability, we iterate over each fact
  std::set< std::string > facts;
  for( auto node : * state )
  {
    //std::cout << * node << std::endl;

    if( ( node->parents.first()->keys.first() == "NOT_OBSERVABLE" ) )
    {
      //std::cout << "is not observable!!" << node->parents.first()->keys.d0 << std::endl;
    }
    else
    {
      std::stringstream ss;
      ss << * node;
      facts.insert( ss.str() );
    }
  }

  return facts;
}

//===========================================================================

static uint _n_get_actions;
static uint _get_actions_time_us;

static uint _n_transitions;
static uint _transition_time_us;

static uint _n_state_str;
static uint _state_str_time_us;

/// root node init
POGraphNode::POGraphNode( rai::Array< std::shared_ptr< FOL_World > > fols, const arr & bs )
  : root_( nullptr )
  , N_( fols.N )
  , folEngines_( fols )
  , folStates_( N_ )
  , resultStates_( N_ )
  //, folAddToStates_( N_ )
  , graph_( std::make_shared< std::list< POGraphNode::ptr > >() )
  //, pHistory_( 1.0 )
  , p_( 1.0 )
  , q_( 1.0 )
  , bs_( bs )
  //, a_( -1 )
  // global search
  , isExpanded_( false )
  , isInfeasible_( false )
  // logic search
  , isTerminal_( false )
  , isSolved_( false )
  , id_( 0 )
{
  for( auto w = 0; w < N_; ++w )
  {
    folEngines_( w )->reset_state();
    folStates_( w ).reset( folEngines_( w )->createStateCopy() );
  }

  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      auto result = folEngines_( w )->getState();

auto start_2 = std::chrono::high_resolution_clock::now();

      auto stateStr = getStateStr( result );++_n_state_str;

auto elapsed_2 = std::chrono::high_resolution_clock::now() - start_2;
long long mcs_2 = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_2).count();
_state_str_time_us += mcs_2;

      resultStates_[ w ] = std::move( stateStr );
    }
  }

  auto wptr = std::shared_ptr<POGraphNode>( this, [](POGraphNode*){} );// TRICK for shared_from_this!!!!!

  graph_->push_back( shared_from_this() );
}

/// child node creation
POGraphNode::POGraphNode( const POGraphNode::ptr & root, double p, double q, const arr & bs,  const std::vector< SymbolicState > & resultStates, uint a )
  : root_( root )
  , N_( root->N_ )
  , folEngines_( root->folEngines_ )
  , folStates_( N_ )
  , resultStates_( resultStates )
  //, folAddToStates_( N_ )
  //, decisions_( N_ )
  , graph_( root->graph_ )
  //, pHistory_( pHistory )
  , p_( p )
  , q_( q )
  , bs_( bs )
  //, a_( a )
  // global search
  , isExpanded_( false )
  , isInfeasible_( false )
  , isTerminal_( false )
  , isSolved_( false )
{
  //std::cout << "-------------------" << std::endl;

  // update the states
  bool isTerminal = true;
  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      rai::String mlrState( resultStates_[ w ].state ); // construct mlr string from std::string
      // logic
      auto fol = folEngines_( w );

      fol->set_state( mlrState );

      folStates_( w ).reset( fol->createStateCopy() );

      bool isSubNodeTerminal = fol->successEnd;
      isTerminal = isTerminal && isSubNodeTerminal;

      if( fol->deadEnd )
      {
        isInfeasible_ = true;
      }
    }
  }
  isTerminal_ = isTerminal;

  if( isTerminal )
  {
    isSolved_ = true;
  }

  // update time
  //auto ls = getWitnessLogicAndState();
  //lastActionReward_ = ls.logic->lastStepReward;
  //prefixReward_ = parent_->prefixReward_ + lastActionReward_;

  // update support size
  std::size_t s = 0;

  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      s++;
    }
  }

  // change this
  id_ = nodeNumber();
}

POGraphNode::L POGraphNode::expand()
{
  POGraphNode::L newNodes;

  //
  auto start = std::chrono::high_resolution_clock::now();
  //

  CHECK( ! isExpanded_, "" );
  if( isTerminal_ )
  {
    return newNodes;
  }

  // get possible actions for the worlds having a non null probability
  // retrieve actions for each world
  uint nActions = 0;
  std::vector< std::vector<FOL_World::Handle> > world_to_actions = getPossibleActions( nActions );

  if( nActions == 0 ) isTerminal_ = true;

  //std::cout << "number of possible actions:" << nActions << std::endl;

  for( auto a = 0; a < nActions; ++a )
  {
    //std::cout << "------------" << std::endl;
    //std::cout << "action:" << a << std::endl;
    std::unordered_map< std::set< std::string >, std::list< uint >, StringSetHash > outcomesToWorlds;
    std::vector< SymbolicState > resultStates( N_ );

    for( auto w = 0; w < N_; ++w )
    {
      if( bs_( w ) > eps() )
      {
        auto logic = folEngines_( w );
        auto state = folStates_( w );
        auto action = world_to_actions[ w ][ a ];
        //logic->addTerminalRule();
        //logic->reset_state();
        CHECK( logic->KB.isDoubleLinked == false, "wrong parametrization!" );

        logic->setState( state.get() );

auto start_1 = std::chrono::high_resolution_clock::now();

        logic->transition( action ); _n_transitions++;

auto elapsed_1 = std::chrono::high_resolution_clock::now() - start_1;
long long mcs_1 = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_1).count();
_transition_time_us += mcs_1;

        auto result             = logic->getState();

auto start_2 = std::chrono::high_resolution_clock::now();

        auto stateStr           = getStateStr( result ); ++_n_state_str;
        auto observableStateStr = getObservableStateStr( result );

auto elapsed_2 = std::chrono::high_resolution_clock::now() - start_2;
long long mcs_2 = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_2).count();
_state_str_time_us += mcs_2;

        resultStates[ w ] = std::move( stateStr );
        outcomesToWorlds[ std::move( observableStateStr ) ].push_back( w );
      }
    }

    // compute the observable facts intersection
    std::set< std::string > intersection = outcomesToWorlds.begin()->first;
    for( auto & outcome = ++outcomesToWorlds.begin(); outcome != outcomesToWorlds.end(); ++outcome )
    {
      auto facts  = outcome->first;
      std::set< std::string > inter;
      std::set_intersection( intersection.begin(), intersection.end(),
                             facts.begin(), facts.end(),
                             std::inserter( inter, inter.begin() ) );

      intersection = std::move( inter );
    }

    // create as many children as outcomes
    POGraphNode::L familiy;
    for( const auto & outcome : outcomesToWorlds )
    {
      const auto & facts  = outcome.first;
      const auto & worlds = outcome.second;

      // update belief state
      arr bs = zeros( N_ );
      double q = 0;
      for( auto w : worlds )
      {
        q      += bs_( w );
        bs( w ) = bs_( w );
      }

      bs = bs / q;

      CHECK( q > 0, "wrong node expansion" );

      // find or create a node for each possible outcome
      POGraphNode::ptr child;
      bool found = false;

      // check for existing node in graph
      for( auto m = graph_->begin(); m != graph_->end(); ++m )
      {
        const auto & a = (*m)->resultStates();
        const auto & b = resultStates;

        if( (*m)->bs() == bs && SymbolicState::equivalent( a, b )  )
        {
          child = *m;
          found = true;
          break;
        }
      }

      // create new node
      if( ! found )
      {
        CHECK( child == nullptr, "a child was found but the pointer is still null!!" );

        CHECK( q <= 1, "impossible probabilities" );
        //CHECK( p_ <= q, "impossible probabilities" );

        child = std::make_shared< POGraphNode >( root(), q * p_, q, bs, resultStates, a );
        graph_->push_back( child );
        newNodes.push_back( child );

        // get the fact not in intersection
        std::set< std::string > differenciatingFacts;
        std::set_difference( facts.begin(), facts.end(), intersection.begin(), intersection.end(),
                             std::inserter( differenciatingFacts, differenciatingFacts.begin() ) );

        child->indicateDifferentiatingFacts( std::move( differenciatingFacts ) );
        //std::cout << "history:" << pHistory << " belief state:" << bs << " family size:" << familiy.d0 << std::endl;
      }

      // add child to created family
      child->addParent( shared_from_this(), a );
      familiy.append( child );
    }

    // check integrity
    double pSum = 0;
    for( const auto & n : familiy ) pSum += n->p();

    CHECK_ZERO( pSum / p() - 1.0, 0.000001, "" );
    //

    families_.append( familiy );

    // indicate and relation
    double one = 0;
    for( const auto & n : familiy )
    {
      n->setAndSiblings( familiy );
      one += n->q();
    }

    CHECK_ZERO( one - 1.0, 0.000001, "wrong probabilities in graph nodes" );
  }

  isExpanded_ = true;

  //
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

  static int n; n++;
  if( n % 100 == 0 ) std::cout << "expansion time:" << ms << " |"
                               << " get actions:" << _n_get_actions << " get actions time(ms):" << _get_actions_time_us / 1000.0 << " |"
                               << " n transitions:" << _n_transitions << " transition time(ms):" << _transition_time_us / 1000.0
                               << " n state->str:" << _n_state_str << " time(ms):" << _state_str_time_us / 1000.0
                               << " families:"<< families_.size() << std::endl;

  _n_transitions = 0;
  _transition_time_us = 0;

  _n_get_actions = 0;
  _get_actions_time_us = 0;

  _n_state_str = 0;
  _state_str_time_us = 0;

  return newNodes;
}

void POGraphNode::setAndSiblings( const POGraphNode::L & siblings )
{
  andSiblings_.clear();

  for( const auto & s : siblings )
  {
    if( s != shared_from_this() ) andSiblings_.append( s );
  }
}

void POGraphNode::addParent( const POGraphNode::ptr & parent, uint action )
{
  parents_.append( parent );
  leadingActions_.append( action );
}

uint POGraphNode::getLeadingActionFrom( const POGraphNode::ptr & parent ) const
{
  CHECK( parent, "no parent!" );

  uint index = parents_.findValue( parent );
  return leadingActions_( index );
}

std::string POGraphNode::getLeadingActionFromStr( const POGraphNode::ptr & parent ) const
{
  CHECK( parent, "no parent!" );

  return parent->actionStr( getLeadingActionFrom( parent ) );
}

std::vector< std::vector<FOL_World::Handle> > POGraphNode::getPossibleActions( uint & nActions ) const
{
  std::vector< std::vector<FOL_World::Handle> > world_to_actions( N_ );

  //std::cout << "------------------------" << std::endl;

  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      auto logic = folEngines_( w );
      auto state = folStates_( w );

      logic->setState( state.get() );

auto start_1 = std::chrono::high_resolution_clock::now();

      auto actions = folEngines_( w )->get_actions(); _n_get_actions++;

auto elapsed_1 = std::chrono::high_resolution_clock::now() - start_1;
long long mcs_1 = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_1).count();
_get_actions_time_us += mcs_1;

      world_to_actions[ w ] = actions;
      nActions = actions.size();

//      for( auto a : actions )
//      std::cout << *a << std::endl;
    }
  }

  return world_to_actions;
}

LogicAndState POGraphNode::getWitnessLogicAndState() const
{
  auto worlds = getPossibleLogicAndStates();

  CHECK( worlds.d0 > 0, "Unable to find a witness logic!!" );

  return worlds.first();
}

rai::Array< LogicAndState > POGraphNode::getPossibleLogicAndStates() const
{
  rai::Array< LogicAndState > worlds;

  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      worlds.append( { folEngines_( w ), folStates_( w ) } );
    }
  }

  return worlds;
}

std::string POGraphNode::actionStr( uint a ) const
{
  const auto & ls = getWitnessLogicAndState();
  ls.logic->reset_state();
  ls.logic->setState( ls.state.get() );

//  std::cout << "state:" << *ls.state.get() << std::endl;

//  auto start_1 = std::chrono::high_resolution_clock::now();

  auto actions = ls.logic->get_actions(); _n_get_actions++;

//  auto elapsed_1 = std::chrono::high_resolution_clock::now() - start_1;
//  long long mcs_1 = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_1).count();
 // _get_actions_time_us += mcs_1;

  std::stringstream ss;

  if( a >= 0 && a < actions.size() )  ss << *actions[a];
  else                                ss << "no actions";

//  std::cout << "action:" << ss.str() << std::endl;

  return ss.str();
}
}
