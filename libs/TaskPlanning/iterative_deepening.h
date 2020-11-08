#pragma once

#include <queue>
#include <string>
#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <policy.h>
#include <task_planner.h>

#include <po_node.h>
#include <node_visitors.h>

namespace tp
{

class IterativeDeepeningPlanner : public TaskPlanner
{
public:
  IterativeDeepeningPlanner();

  // modifiers
  void setFol( const std::string & folDescription ) override;
  void solve() override;
  void integrate( const Policy::ptr & policy ) override;

  void setDmax( uint dmax ) { dmax_ = dmax; }

  // getters
  Policy::ptr getPolicy() const override;
  MotionPlanningOrder getPlanningOrder() const override;
  bool      terminated () const override { return solved(); }

private:
  bool solved() const { return root_->isSolved(); }

private:
  void solveBreadthFirst();
  void breadthFirstExpand( uint d );

  void solveIterativeDepthFirst();
  PONode::ptr iterativeDepthFirstExpand( const PONode::ptr &, uint e );
  uint n_exp_ = 0;

  void extractSolutions();

private:
  void checkIntegrity();

private:
  // state
  rai::Array< std::shared_ptr<FOL_World> > folWorlds_;
  arr bs_;

  PONode::ptr root_;

  uint dmax_;

  std::set< PONode::ptr > checked_;
  std::queue< PONode::ptr > queue_;
  std::list< PONode::ptr > terminals_;

  const rai::String beliefStateTag_  = "BELIEF_START_STATE";
};

}
