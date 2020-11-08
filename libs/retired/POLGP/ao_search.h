#pragma once

#include <list>
#include <set>

#include <Logic/fol_mcts_world.h>
#include <Kin/kinViewer.h>
#include "polgp_node.h"
#include "policy.hpp"

// sort nodes so that the ones with the biggest rewards are first
//struct POLGPNodeCompare : public std::binary_function<POLGPNode*, POLGPNode*, bool>
//{
//    bool operator()( POLGPNode* lhs, POLGPNode* rhs) const
//    {
//        return ! ( lhs->expecteTotalReward() == rhs->expecteTotalReward() ) && ( lhs->expecteTotalReward() > rhs->expecteTotalReward() );
//    }
//};

class AOSearch
{
public: // public methods
  AOSearch( const KOMOFactory & );

  // modifiers
  void prepareFol( const std::string & folDescription );
  void prepareKin( const std::string & kinDescription );
  void prepareTree();
  void prepareDisplay();

  //void registerGeometricLevel( GeometricLevelFactoryBase::ptr const& factory );

  void solveSymbolically();
  bool isPolicyFringeEmpty() const { return currentPolicyFringeInitialized_ && ( currentPolicyFringe_.size() == 0 ); }
  void generateAlternativeSymbolicPolicy();
  void revertToPreviousPolicy();

  void solveGeometrically();

  void optimizePoses();
  void optimizePaths();
  void optimizeJointPaths();

  void updateDisplay( const WorldID & w, bool poses, bool seqs, bool paths );

private:
  void optimizePosesFrom( POLGPNode * );

public:
  // getters
  bool isSymbolicallySolved() const { return root_->isSymbolicallySolved(); }
  bool isPoseSolved() const { return root_->isPoseSolved(); }
  bool isPathSolved() const { return root_->isPathSolved(); }
  bool isJointPathSolved() const { return root_->isJointPathSolved(); }

  uint alternativeNumber() const { return alternativeNumber_; }

  // helpers
  tmp::Policy::ptr getPolicy() const;

  void printPolicy( const std::string & name, bool generatePng = true ) const;
  void printSearchTree( const std::string & name, bool generatePng = true ) const;
  void printPolicy( std::iostream & ) const;
  void printSearchTree( std::iostream & ) const;

private:
  rai::Array< POLGPNode * > getNodesToExpand() const;   // go along the best solution so far and accumulates the nodes that haven't been expanded, it goes up to the "deepest nodes" of the temporary path
  rai::Array< POLGPNode * > getNodesToExpand( POLGPNode * ) const;

  rai::Array< POLGPNode * > getTerminalNodes() const;
  rai::Array< POLGPNode * > getTerminalNodes( POLGPNode * ) const;

  POLGPNode * getTerminalNode( const WorldID & w ) const;

  void printPolicy( POLGPNode * node, std::iostream & ) const;
  void printSearchTree( POLGPNode * node, std::iostream & ss ) const;

private:
  //FOL_World fol;      // first order logic symbols
  rai::Array< std::shared_ptr<FOL_World> > folWorlds_;
  rai::Array< std::shared_ptr< const rai::KinematicWorld > > kinematics_;

  arr bs_;
  POLGPNode * root_; // root and "current" node

  //
  //std::set< POLGPNode *, POLGPNodeCompare > openFringe_;        // fringe of the current search tree
  //std::set< POLGPNode *, POLGPNodeCompare > alternativeNodes_;  // when looking for alternatives, use the node in this set, when it becomes empty, backupthe current open fringe
  std::set< POLGPNodeL > currentPolicyFringe_;
  bool currentPolicyFringeInitialized_;
  uint alternativeNumber_;
  POLGPNode * alternativeStartNode_;
  POLGPNodeL nextFamilyBackup_;
  std::set< POLGPNodeL > currentPolicyFringeBackup_;  
  //

  // geometric levels
  const KOMOFactory & komoFactory_;
  rai::Array< GeometricLevelFactoryBase::ptr > geometricLevelFactories_;

  // display
  rai::Array< std::shared_ptr< OrsPathViewer > > poseViews_;
  rai::Array< std::shared_ptr< OrsPathViewer > > pathViews_;

  // params
  const rai::String beliefStateTag_  = "BELIEF_START_STATE";
};

//===========================================================================

//class SearchSpaceTree{
//public: // public members
//  //FOL_World fol;      // first order logic symbols
//  rai::Array< std::shared_ptr<FOL_World> > folWorlds_;
//  arr bs_;

//  PartiallyObservableNode *root,*node; // root and "current" node

//  OrsPathViewer poseView;
//  OrsPathViewer seqView;
//  OrsPathViewer pathView;

//  rai::Array<PartiallyObservableNode*> mcFringe;
//  rai::Array<PartiallyObservableNode*> terminals;
//  rai::Array<PartiallyObservableNode*> poseFringe;
//  rai::Array<PartiallyObservableNode*> seqFringe;
//  rai::Array<PartiallyObservableNode*> pathFringe;
//  rai::Array<PartiallyObservableNode*> done;

//public: // public methods
//  SearchSpaceTree( const KOMOFactory & );

//  // modifiers
//  void prepareKin( const std::string & kinematicDescription );
//  void prepareFol( const std::string & folDescription );
//  void prepareTree();
//  void prepareDisplay();

//  void updateDisplay();

//  // technically const but let as such, as the semantic is not const
//  bool execRandomChoice();
//  bool execChoice( rai::String cmd );

//  // const
//  void displayTree() const{ int r=system("evince z.pdf &");  if(r) LOG(-1) <<"could not startup evince"; }
//  void printChoices() const;
//  rai::String queryForChoice() const;

//private: // private methods

//private: // private members
//  rai::KinematicWorld kin;

//  KOMOFactory komoFactory_;

//  bool autoCompute = false;
//};

//============free functions==============================================

//double poseHeuristic(PartiallyObservableNode* n);
//double mcHeuristic(PartiallyObservableNode* n);
//double seqHeuristic(PartiallyObservableNode* n);
//double poseCost(PartiallyObservableNode* n);
//double seqCost(PartiallyObservableNode* n);
//double pathHeuristic(PartiallyObservableNode* n);
//double pathCost(PartiallyObservableNode* n);
//PartiallyObservableNode* getBest(rai::Array<PartiallyObservableNode*>& fringe, double heuristic(PartiallyObservableNode*));
//PartiallyObservableNode* popBest(rai::Array<PartiallyObservableNode*>& fringe, double heuristic(PartiallyObservableNode*));

//typedef ActionNode MNode;

//double poseHeuristic(MNode* n);
//double mcHeuristic(MNode* n);
//double seqHeuristic(MNode* n);
//double poseCost(MNode* n);
//double seqCost(MNode* n);
//double pathHeuristic(MNode* n);
//double pathCost(MNode* n);
//MNode* getBest(rai::Array<MNode*>& fringe, double heuristic(MNode*));
//MNode* popBest(rai::Array<MNode*>& fringe, double heuristic(MNode*));
