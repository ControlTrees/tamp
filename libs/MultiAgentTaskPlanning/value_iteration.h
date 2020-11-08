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

namespace matp
{

class ValueIterationAlgorithm
{
public:
  static std::vector< double > process( const DecisionGraph & decisionGraph, Rewards & rewards );
};

} // namespace matp
