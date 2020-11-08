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

#pragma once

#include <set>

#include <Kin/kin.h>
#include <Logic/fol_mcts_world.h>
#include <Logic/fol.h>
#include "node_visitor.h"
#include "utility.h"

struct PlainMC;
struct MCStatistics;

namespace tp
{

//===========================================================================

class PONode : public std::enable_shared_from_this< PONode > // note: public inheritance
{
  friend class NodeVisitorBase;
public:
  typedef std::shared_ptr< PONode > ptr;
  typedef rai::Array< PONode::ptr > L;
  typedef rai::Array< rai::Array<PONode::ptr > > LL;

public:
  /// root node init
  PONode( rai::Array< std::shared_ptr< FOL_World > > fols, const arr & bs );

  /// child node creation
  PONode( const PONode::ptr & parent, double pHistory, const arr & bs, uint a );

  // modifiers
  void expand();
  void setAndSiblings( const PONode::L & siblings );
  void setBestFamily( const PONode::L & f ) { bestFamily_ = f; expectedBestA_ = f( 0 )->a_; }
  void generateMCRollouts( uint num, int stepAbort, uint maxHorizon = 100 );
  void backTrackBestExpectedPolicy( PONode::ptr until_node = nullptr ); // backtrack up to the node node, per default, backup up to root

  void backTrackSolveStatus(); // backtrack up to the node node, per default, backup up to root

  void labelInfeasible(); ///< sets the infeasible label, should remove all children?
  //void resetSymbolicallySolved() { isSymbolicallySolved_ = false; }

  void acceptVisitor( NodeVisitorBase & visitor ) { visitor.visit( shared_from_this() ); } // overkill here, visitor design pattern usefull if we have a hierarchy of class!
  //void labelInfeasible();

  // getters
  PONode::ptr parent() const { return parent_; }
  bool isExpanded() const { return isExpanded_; }
  PONode::LL families() const { return families_; }
  bool isTerminal() const { return isTerminal_; }
  bool isSolved() const { return   isSolved_; }
  rai::Array< std::shared_ptr<Graph> > folStates() const { return folStates_; }

  uint N() const { return N_; }
  int id() const { return id_; }
  PONode::L bestFamily() const { return bestFamily_; }
  PONode::L andSiblings() const { return andSiblings_; }
  double pHistory() const { return pHistory_; }
  bool isRoot() const { return parent_ == nullptr; }
  arr bs() const { return bs_; }

  PONode::L getTreePath();
  PONode::L getTreePathFrom( const PONode::ptr & start );
  FOL_World::Handle & decision( uint w ) const { return decisions_( w ); }

  double time() const { return time_; }
  uint   depth() const{ return d_; }
  double prefixReward() const { return prefixReward_; }
  double lastReward() const { return lastActionReward_; }
  double value() const { return value_; }
  double expecteFutureReward() const { return value_ - prefixReward_; }

  // utility
  std::string bestActionStr() const { return actionStr( expectedBestA_ ); }
  std::string leadingActionStr() const { return parent_->actionStr( a_ ); }
  void indicateDifferentiatingFacts( const std::set< std::string > & facts ) { differentiatingFacts_ = facts; }
  std::set< std::string > differentiatingFacts() const { return differentiatingFacts_; }
  std::shared_ptr< MCStatistics > mcStats() const { return mcStats_; }

private:
  // utility
  uint getPossibleActionsNumber() const;
  std::vector< std::vector<FOL_World::Handle> > getPossibleActions( uint & nActions ) const;
  LogicAndState getWitnessLogicAndState() const;
  template < typename T > T getWitnessElem( const rai::Array< T > array ) const
  {
    CHECK( array.d0 == N_, "wrong dimensions!" );
    for( auto w = 0; w < N_; ++w )
    {
      if( bs_( w ) > std::numeric_limits< double >::epsilon() )
      {
        return array( w );
      }
    }
  }

  rai::Array< LogicAndState > getPossibleLogicAndStates() const;
  std::string actionStr( uint ) const;

private:
  PONode::ptr parent_;

  // members for symbolic search
  uint N_;                                                                    ///< number of possible worlds
  rai::Array< std::shared_ptr<FOL_World> > folWorlds_;
  rai::Array< std::shared_ptr<Graph> >     folStates_;                        ///< INITIAL fol state, state when the PARENT action has been executed

  double pHistory_;
  arr    bs_;

  int a_;                                         ///< action id that leads to this node
  rai::Array< FOL_World::Handle > decisions_;     ///< actions leading to this node ( one for each logic )

  uint d_;                                        ///< decision depth/step of this node
  double time_;                                   ///< real time, root = 0, represents the end of the parent action

  PONode::L  andSiblings_;            /// at the same depth!
  PONode::LL families_;
  std::set< std::string > differentiatingFacts_;  ///< used only for debugging purposes

  rai::Array< std::shared_ptr< PlainMC > > rootMCs_;
  std::shared_ptr< MCStatistics > mcStats_;       /// best results (future rewards) of the rollouts

  double lastActionReward_;                       ///  reward of the action leading to this node
  double prefixReward_;                           ///  this is the (certain) rewards of the prefix decisions
  double value_;                   ///  the expected total rewards
  //double expectedFutureReward_;                   ///  the expected future reward ?

  int expectedBestA_;                             ///  expected next best action
  PONode::L bestFamily_;

  //-- global search
  bool isExpanded_;
  bool isInfeasible_;

  //-- logic search
  bool isTerminal_;           /// all the fol of this node are terminated
  bool isSolved_;             /// the children of this node are all solved

  //--
  int id_;
};

namespace utility
{
  // free functions
  PONode::ptr getTerminalNode( const PONode::ptr &, const WorldID & w );
  void   gatherPolicyFringe( const PONode::ptr &s, std::set< rai::Array< PONode::ptr> > & );
}

}
