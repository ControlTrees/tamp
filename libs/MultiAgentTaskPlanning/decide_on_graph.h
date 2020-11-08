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

class DecideOnGraphAlgorithm
{
public:
  static DecisionGraph process( const DecisionGraph & decisionGraph, std::vector< double > & values, Rewards & rewards );
};

} // namespace matp
