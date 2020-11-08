#pragma once

#include <stdexcept>

#include <queue>
#include <string>
#include <unordered_map>

#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <skeleton.h>
#include <task_planner.h>

#include <logic_parser.h>
#include <decision_graph.h>

#include <value_iteration.h>
#include <decide_on_graph.h>

namespace matp
{

std::vector< std::string > decisionArtifactToKomoArgs( const std::string & artifact );

class GraphPlanner : public TaskPlanner
{
public:
  // modifiers
  virtual void setFol( const std::string & descrition ) override;
  virtual void solve() override;
  virtual void integrate( const Policy & policy ) override;

  // getters
  virtual bool terminated() const override;
  Policy getPolicy() const override;
  double reward( uint from, uint to  ) const; // exposed for testing purpose only
  double R0() const { return r0_; }

  // other modifiers
  void setR0( double r0 ) { r0_ = r0; }
  void setMaxDepth( uint d ) { maxDepth_ = d; }
  void buildGraph( bool graph = false );
  void initializeRewards();
  void saveGraphToFile( const std::string & filename ) const { graph_.saveGraphToFile( filename ); }
  void saveDecidedGraphToFile( const std::string & filename ) const { decidedGraph_.saveGraphToFile( filename ); }

  // other getters
  DecisionGraph decisionGraph() const { return graph_; }
  DecisionGraph decidedDecisionGraph() const { return decidedGraph_; }
  std::vector< double > values() const { return values_; }
  uint agentNumber() const { return parser_.agentNumber(); }

  // stand-alone
  PolicyNodeData decisionGraphtoPolicyData( const NodeData & n, uint ) const;

private:
  void valueIteration();
  void decideOnDecisionGraphCopy();
  void buildPolicy();
  uint fromToIndex( uint from, uint to ) const;

private:
  LogicParser parser_;
  DecisionGraph graph_;
  Policy policy_;

  // graph expansion
  uint maxDepth_ = 3;

  // value iteration
  double r0_ = -1;
  mutable Rewards rewards_;//std::vector< double > rewards_; // current state of rewards
  std::vector< double > values_;
  DecisionGraph decidedGraph_;

};

} // namespace matp
