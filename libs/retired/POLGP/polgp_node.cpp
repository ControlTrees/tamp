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


#include "polgp_node.h"

#include <unordered_map>

#include <list>

#include <boost/algorithm/string/replace.hpp>

#include <MCTS/solver_PlainMC.h>

#include "geometric_levels.h"

#define DEBUG(x) //x
#define DEL_INFEASIBLE(x) x

uint COUNT_kin=0;
uint COUNT_evals=0;
uint COUNT_poseOpt=0;
uint COUNT_seqOpt=0;
uint COUNT_pathOpt=0;

//=====================free functions======================
static double eps() { return std::numeric_limits< double >::epsilon(); }
static double m_inf() { return -std::numeric_limits< double >::max(); }

struct stringSetHash {
size_t operator()( const std::set< std::string > & facts ) const
{
  std::string cont;
  for( auto s : facts )
    cont += s;

  return std::hash<std::string>()( cont );
}
};

static std::string toStdString( Node * node )
{
  std::stringstream ss;
  ss << * node;
  return ss.str();
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

static int nodeNumber = 0;

/// root node init
POLGPNode::POLGPNode( rai::Array< std::shared_ptr< FOL_World > > fols, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & kins, const arr & bs, const KOMOFactory & komoFactory )
  : parent_( nullptr )
  , N_( fols.N )
  , folWorlds_( fols )
  , folStates_( N_ )
  //, folAddToStates_( N_ )
  , startKinematics_( kins )
  , effKinematics_( N_ )
  , pHistory_( 1.0 )
  , bs_( bs )
  , a_( -1 )
  , d_( 0 )
  , time_( 0.0 )
  // global search
  , isExpanded_( false )
  , isInfeasible_( false )
  //, isTerminal_( false )
  //, isSolved_( false )
  // logic search
  , isSymbolicallyTerminal_( false )
  , isSymbolicallySolved_( false )
  , rootMCs_( N_ )
  , mcStats_( new MCStatistics )
  , lastActionReward_( 0 )
  , prefixReward_( 0 )
  , expectedReward_( m_inf() )
  , expectedBestA_( -1 )
  //, komoFactory_( komoFactory )
  // poseOpt
  , poseProblem_( new PoseLevelType( this, komoFactory ) )
  // pathOpt
  , pathProblem_( new PathLevelType( this, komoFactory ) )
  // jointPathOpt
  , jointProblem_( new JointPathLevelType( this, komoFactory ) )
  //
  , id_( 0 )
{
  for( auto w = 0; w < N_; ++w )
  {
    folWorlds_( w )->reset_state();
    folStates_( w ).reset( folWorlds_( w )->createStateCopy() );
    //folAddToStates_( w ) = nullptr;
    rootMCs_( w ).reset( new PlainMC( *folWorlds_( w ) ) );
    rootMCs_( w )->verbose = 0;
  }

  for( auto w = 0; w < N_; ++w )
  {
    effKinematics_( w ) = rai::KinematicWorld( * startKinematics_( w ) );
  }

  std::size_t s = 0;

  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      s++;
    }
  } 
}

/// child node creation
POLGPNode::POLGPNode(POLGPNode *parent, double pHistory, const arr & bs, uint a )
  : parent_( parent )
  , N_( parent_->N_ )
  , folWorlds_( parent->folWorlds_ )
  , folStates_( N_ )
  //, folAddToStates_( N_ )
  , startKinematics_( parent->startKinematics_ )
  , effKinematics_( parent->effKinematics_ )
  , decisions_( N_ )
  , pHistory_( pHistory )
  , bs_( bs )
  , a_( a )
  , d_( parent->d_ + 1 )
  // global search
  , isExpanded_( false )
  , isInfeasible_( false )
  //, isTerminal_( false )
  //, isSolved_( false )
  // logic search
  , isSymbolicallyTerminal_( false )
  , isSymbolicallySolved_( false )
  , rootMCs_( parent->rootMCs_ )
  , mcStats_( new MCStatistics )
  , lastActionReward_( 0 )
  , prefixReward_( 0 )
  , expectedReward_( m_inf() )
  , expectedBestA_ (-1 )
  //, komoFactory_( parent->komoFactory_ )
  // poseOpt
  , poseProblem_( new PoseLevelType( this, parent_->poseProblem_->komoFactory() ) )
  // pathOpt
  , pathProblem_( new PathLevelType( this, parent_->pathProblem_->komoFactory() ) )
  // jointPathOpt
  , jointProblem_( new JointPathLevelType( this, parent_->jointProblem_->komoFactory() ) )
{
  // update the states
  bool isTerminal = true;
  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      // logic
      auto fol = folWorlds_( w );
      //fol->reset_state();
      fol->setState( parent->folStates_( w ).get(), parent_->d_ );
      auto actions = fol->get_actions();

      fol->transition( actions[ a_ ] );

      folStates_( w ).reset( fol->createStateCopy() );

      bool isSubNodeTerminal = fol->successEnd;
      isTerminal = isTerminal && isSubNodeTerminal;

//      if( isTerminal )
//        std::cout << "found terminal " << std::endl;

      if( fol->deadEnd )
      {
        isInfeasible_ = true;
      }
      //std::cout << *folStates_( w ) << std::endl;

      //folAddToStates_( w ) = nullptr;

      decisions_( w ) = actions[ a_ ];
    }
  }
  isSymbolicallyTerminal_ = isTerminal;

  if( isTerminal )
  {
    isSymbolicallySolved_ = true;
  }

  // update time
  auto ls = getWitnessLogicAndState();
  time_ = parent_->time_ + ls.logic->lastStepDuration;
  lastActionReward_ = ls.logic->lastStepReward;
  prefixReward_ = parent_->prefixReward_ + lastActionReward_;

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
  nodeNumber++;
  id_ = nodeNumber;
}

void POLGPNode::expand()
{
  // debug
  //std::cout << std::endl;
  //std::cout << "POLGPNode::expand().." << std::endl;
  //auto ls = getWitnessLogicAndState();
  //std::cout << "observable state:" << getObservableStateStr( ls.state.get() ) << std::endl;
  //

  CHECK( ! isExpanded_, "" );
  if( isSymbolicallyTerminal_ )
    return;

  // get possible actions for the worlds having a non null probability
  auto nActions = getPossibleActionsNumber();

  //std::cout << "number of possible actions:" << nActions << std::endl;

  if( nActions == 0 )
    isSymbolicallyTerminal_ = true;

  for( auto a = 0; a < nActions; ++a )
  {
    //std::cout << "------------" << std::endl;
    //std::cout << "action:" << a << std::endl;
    std::unordered_map< std::set< std::string >, std::list< uint >, stringSetHash > outcomesToWorlds;

    for( auto w = 0; w < N_; ++w )
    {
      if( bs_( w ) > eps() )
      {
        auto logic = folWorlds_( w );
        auto state = folStates_( w );

        logic->setState( state.get() );

        auto actions = logic->get_actions();
        auto action = actions[ a ];

        logic->transition( action );

        auto result = logic->getState();
        auto observableStateStr = getObservableStateStr( result );

        outcomesToWorlds[ observableStateStr ].push_back( w );
      }
    }

    //std::cout << outcomesToWorlds.size() << " possible outcomes" << std::endl;

    // compute the observable facts intersection
    std::set< std::string > intersection = outcomesToWorlds.begin()->first;
    for( auto outcome = ++outcomesToWorlds.begin(); outcome != outcomesToWorlds.end(); ++outcome )
    {
      auto facts  = outcome->first;
      std::set< std::string > inter;
      std::set_intersection( intersection.begin(), intersection.end(),
                             facts.begin(), facts.end(),
                             std::inserter( inter, inter.begin() ) );
      intersection = inter;
    }

    // create as many children as outcomes
    rai::Array< POLGPNode * > familiy;
    for( auto outcome : outcomesToWorlds )
    {
      auto facts  = outcome.first;
      auto worlds = outcome.second;

      // update belief state
      arr bs = zeros( N_ );
      double pWorld = 0;
      for( auto w : worlds )
      {
        pWorld += bs_( w );
        bs( w ) = bs_( w );
      }

      bs = bs / pWorld;

      CHECK( pWorld > 0, "wrong node expansion" );

      // create a node for each possible outcome

      auto n = new POLGPNode( this, pWorld * pHistory_, bs, a );
      familiy.append( n );

      // get the fact not in intersection
      std::set< std::string > differenciatingFacts;
      std::set_difference( facts.begin(), facts.end(), intersection.begin(), intersection.end(),
                           std::inserter(differenciatingFacts, differenciatingFacts.begin() ) );

      n->indicateDifferentiatingFacts( differenciatingFacts );
      //std::cout << "history:" << pHistory << " belief state:" << bs << " family size:" << familiy.d0 << std::endl;
    }

    // check integrity
    double pSum = 0;
    for( auto n : familiy ) pSum += n->pHistory();

    CHECK_ZERO( pSum / pHistory() - 1.0, 0.000001, "" );
    //

    families_.append( familiy );

    // indicate and relation
    for( auto n : familiy )
    {
      n->setAndSiblings( familiy );
    }
  }

  isExpanded_ = true;
}

void POLGPNode::setAndSiblings( const rai::Array< POLGPNode * > & siblings )
{
  for( auto s : siblings )
  {
    if( s != this )
    andSiblings_.append( s );
  }
}

void POLGPNode::generateMCRollouts( uint num, int stepAbort )
{
  //std::cout << "POLGPNode::generateMCRollouts.." << std::endl;
  // do rollouts for each possible worlds
  auto treepath = getTreePath();

  arr R;  // rewards of all the rollouts

  for( auto w = 0; w < N_; ++w )
  {
    auto fol = folWorlds_( w );
    auto state = folStates_( w );
    auto rootMC = rootMCs_( w );
    // retrieve history
    if( bs_( w ) > eps() )
    {
      rai::Array<MCTS_Environment::Handle> prefixDecisions( treepath.N-1 );

      for( uint i=1 ; i < treepath.N; i++ )
      {
        prefixDecisions(i-1) = treepath(i)->decision( w );
      }

      for( uint k=0; k < num; ++k )
      {
        fol->reset_state();
        double prefixReward = rootMC->initRollout( prefixDecisions );      
        fol->setState( state.get() );
        double r = rootMC->finishRollout( stepAbort );
        R.append( bs_( w ) * r );

        CHECK( prefixReward_ == prefixReward, "" );
      }
    }
  }

  // save result
  double averageReward = 0; // averaged over the worlds
  for( auto r: R )
  {
    averageReward += r;
  }

  averageReward /= num;     // normalize by the number of rollouts
  mcStats_->add( averageReward );

  // commit result
  expectedReward_ = mcStats_->X.first();

  //std::cout << "average reward:" << expectedReward_ << std::endl;
}

void POLGPNode::backTrackBestExpectedPolicy( POLGPNode * node )
{
  if( isSymbolicallyTerminal() )
  {
    expectedReward_ = prefixReward_;  // it means that the future rewards is = 0 ( already terminal )
    expectedBestA_  = -1;
  }
  else
  {
    struct familyStatusType { double reward; bool solved; };
    rai::Array< familyStatusType > familyStatus( families_.d0 );

    // find best family
    // compute cost of each family
    for( auto i = 0; i < families_.d0; ++i )
    {
      double familyReward = 0;
      bool familySolved = true;
      for( auto c : families_( i ) )
      {
        //CHECK( c->lastActionReward_ == c->getWitnessLogicAndState().logic->lastStepReward, "" );
        familyReward += c->pHistory_ / pHistory_ * c->expectedReward_;
        familySolved = familySolved && c->isSymbolicallySolved_;
      }

      familyStatus( i ) = { familyReward, familySolved };
    }

    // sort
    double bestReward = m_inf();
    int bestFamilyId = -1;
    for( auto i = 0; i < families_.d0; ++i )
    {
      if( familyStatus( i ).reward >= bestReward )
      {
        bestReward = familyStatus( i ).reward;
        bestFamilyId = i;
      }
    }

    // retrieve best decision id
    bestFamily_ = families_( bestFamilyId );
    uint bestA = bestFamily_.first()->a_;
    mcStats_->add( bestReward ); // this one is more informed!
    expectedReward_ = mcStats_->X.first();
    expectedBestA_ = bestA;
    isSymbolicallySolved_ = familyStatus( bestFamilyId ).solved;

    // check
    //std::cout << familyRewards << std::endl;
    //std::cout << actionStr( bestA ) << std::endl;
    //std::cout << "best family size:" << bestFamily_.d0 << " solved?" << familyStatus( bestFamilyId ).solved << std::endl;
    //
  }

  if( parent_ && this != node )
  {
    parent_->backTrackBestExpectedPolicy();
  }
}

void POLGPNode::registerGeometricLevel( GeometricLevelBase::ptr const& level )
{
  geometricLevels_[ level->name() ] = level;
}

void POLGPNode::solvePoseProblem()  // solve all poses from root to this node
{
  poseProblem_->solve();
}

void POLGPNode::solvePathProblem()  // solve all poses from root to this node
{
  pathProblem_->solve();
}

void POLGPNode::solveJointPathProblem()  // solve all poses from root to this node
{
  jointProblem_->solve();
}

void POLGPNode::labelInfeasible()
{
  // set flag and badest reward
  mcStats_->clear();
  isInfeasible_ = true;
  expectedReward_ = m_inf();

  // we reset the rollouts, all the parents rollouts are potentially wrong ( too optimistic ).
  for( auto parent = parent_; parent; parent = parent->parent() )
  {
    parent->mcStats_->clear();
  }

  // delete children nodes
//  for( auto children : families_ )
//  {
//    DEL_INFEASIBLE( children.clear(); )
//  }
//  families_.clear();

  // backtrack results
  if( parent_ )
  {
    parent_->backTrackBestExpectedPolicy();
  }
  //-- remove children
//  ActionNodeL tree;
//  getAllChildren(tree);
//  for(ActionNode *n:tree) if(n!=this) delete n; //TODO: memory leak!
//  for( auto children : families_ )
//  {
//    DEL_INFEASIBLE( children.clear(); )
//  }
//  families_.clear();

//  //-- add INFEASIBLE flag to fol
//  auto folDecision = folStates_( w )->getNode("decision");
//  NodeL symbols = folDecision->parents;
//  symbols.prepend( folWorlds_( w )->KB.getNode({"INFEASIBLE"}));

////  cout <<"\n *** LABELLING INFEASIBLE: "; listWrite(symbols); cout <<endl;
//  //-- find the right parent...
//  POLGPNode* node = this;
//  while( node->parent_ ){
//    bool stop=false;
//    for(Node *fact:node->folStates_( w )->list()){
//      if(fact->keys.N && fact->keys.last()=="block"){
//        if(tuplesAreEqual(fact->parents, symbols)){
//          CHECK(fact->isOfType<bool>() && fact->keys.first()=="block", "");
//          stop=true;
//          break;
//        }
//      }
//    }
//    if(stop) break;
//    node = node->parent_;
//  }

//  if( ! node->folAddToStates_( w ) )
//  {
//    node->folAddToStates_( w ) = &folWorlds_( w )->KB.newSubgraph({"ADD"}, {node->folStates_( w )->isNodeOfGraph})->value;
//  }
//  node->folAddToStates_( w )->newNode<bool>({}, symbols, true);

////  ActionNode *root=getRoot();
//  node->recomputeAllFolStates();
//  node->recomputeAllMCStats(false);
}

rai::Array< POLGPNode * > POLGPNode::getTreePath()
{
  rai::Array< POLGPNode * > path;
  POLGPNode * node = this;
  for(;node;){
    path.prepend(node);
    node = node->parent_;
  }
  return path;
}

rai::Array< POLGPNode * > POLGPNode::getTreePathFrom( POLGPNode * start )
{
  rai::Array< POLGPNode * > subPath;

  POLGPNode * node = this;
  do
  {
    subPath.prepend( node );
    node = node->parent_;

//    if( node == start )
//      std::cout << "node == start " << std::endl;

  } while ( ( node != start ) && node );

  return subPath;
}

uint POLGPNode::getPossibleActionsNumber() const
{
  auto logicAndState = getWitnessLogicAndState();

  logicAndState.logic->setState( logicAndState.state.get(), d_ );

  auto actions = logicAndState.logic->get_actions();

  return actions.size();
}

LogicAndState POLGPNode::getWitnessLogicAndState() const
{
  auto worlds = getPossibleLogicAndStates();

  CHECK( worlds.d0 > 0, "Unable to find a witness logic!!" );

  return worlds.first();
}

rai::Array< LogicAndState > POLGPNode::getPossibleLogicAndStates() const
{
  rai::Array< LogicAndState > worlds;

  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      worlds.append( { folWorlds_( w ), folStates_( w ) } );
    }
  }

  return worlds;
}

std::string POLGPNode::actionStr( uint a ) const
{
  auto ls = getWitnessLogicAndState();
  ls.logic->reset_state();
  ls.logic->setState( ls.state.get() );
  auto actions = ls.logic->get_actions();

  std::stringstream ss;

  if( a >= 0 && a < actions.size() )
    ss << *actions[a];
  else
    ss << "no actions";

  return ss.str();
}

namespace utility
{
//====free functions============//
POLGPNode * getTerminalNode( POLGPNode * n, const WorldID & w )
{
  POLGPNode * node = nullptr;
  if( n->isSymbolicallyTerminal() )
  {
    CHECK( n->bs()( w.id() ) > eps(), "bug in getTerminalNode function, the belief state of the found node is invalid!" );
    node = n;
  }
  else
  {
    for( auto c : n->bestFamily() )
    {
      if( c->bs()( w.id() ) > eps() )
      {
        node = getTerminalNode( c, w );
        break;
      }
    }
  }

  return node;
}

void gatherPolicyFringe( POLGPNode * node, std::set< rai::Array< POLGPNode * > > & fringe )
{
  for( auto f : node->families() )
  {
    if( f != node->bestFamily() )
    {
      fringe.insert( f );
    }
    else
    {
      for( auto c : f )
      {
        gatherPolicyFringe( c, fringe );
      }
    }
  }
}
}

//===========================================================================

RUN_ON_INIT_BEGIN(manipulationTree)
POLGPNodeL::memMove = true;
ActionNodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
