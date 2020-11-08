#pragma once

#include <queue>
#include <string>
#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <skeleton.h>
#include <task_planner.h>

#include <po_graph.h>
#include <node_visitors.h>
#include <po_djikstra.h>

namespace tp
{

class GraphSearchPlanner : public TaskPlanner
{
public:

  // modifiers
  void setFol( const std::string & folDescription ) override;
  void solve() override;
  void integrate( const Policy::ptr & policy ) override;

  void buildGraph();
  void setInitialReward( double r ) { initialReward_ = r; }

  // getters
  POGraph::ptr getGraph() const { return graph_; }
  POWeightedGraph::ptr getWeightedGraph() const { return weightedGraph_; }
  rai::Array< std::shared_ptr<FOL_World> > getFolEngines() const { return folEngines_; }
  Policy::ptr getPolicy() const override;
  MotionPlanningOrder getPlanningOrder() const override;
  bool      terminated () const override { return policy_ != nullptr; }

  // utility
  void saveGraphToFile( const std::string & filename );

private:
  void buildGraphImpl();
  void yen( uint k );   // generates a set of policies

  uint n_exp_ = 0;

private:
  void checkIntegrity();

private:
  // state
  rai::Array< std::shared_ptr<FOL_World> > folEngines_;
  arr bs_;
  POGraph::ptr graph_;
  POWeightedGraph::ptr weightedGraph_;

  // dynamic programming
  double initialReward_ = -1;
  Skeleton skeleton_;

  // constants
  const rai::String beliefStateTag_  = "BELIEF_START_STATE";
};

}
