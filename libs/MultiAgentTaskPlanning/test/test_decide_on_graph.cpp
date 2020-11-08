#include <graph_planner.h>

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

using namespace matp;

// DecideOnGraph
TEST(DecideOnGraph, CorrectDecisionEvenWithInfiniteReward) {
  LogicEngine engine;
  std::vector< std::string > startStates;
  std::vector< double > egoBeliefState;
  DecisionGraph graph( engine, startStates, egoBeliefState );

  auto root = graph.root();

  // branch 1
  auto child_o_1 = root->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION) );
  graph._addNode( child_o_1 );
  graph._addEdge( child_o_1->id(), root->id(), 1.0, "" );
  auto child_a_1 = child_o_1->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, false, 0, NodeData::NodeType::ACTION) );
  graph._addNode( child_a_1 );
  graph._addEdge( child_a_1->id(), child_o_1->id(), 1.0, "" );
  auto child_o_2 = child_a_1->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION) );
  graph._addNode( child_o_2 );
  graph._addEdge( child_o_2->id(), child_a_1->id(), 1.0, "" );
  auto child_a_2 = child_o_2->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, true, 0, NodeData::NodeType::ACTION) );
  graph._addNode( child_a_2 );
  graph._addEdge( child_a_2->id(), child_o_2->id(), 1.0, "" );

  // branch 2
  auto child_o_10 = root->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION) );
  graph._addNode( child_o_10 );
  graph._addEdge( child_o_10->id(), root->id(), 1.0, "" );
  auto child_a_10 = child_o_10->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, false, 0, NodeData::NodeType::ACTION) );
  graph._addNode( child_a_10 );
  graph._addEdge( child_a_10->id(), child_o_10->id(), 1.0, "" );
  auto child_o_20 = child_a_10->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION) );
  graph._addNode( child_o_20 );
  graph._addEdge( child_o_20->id(), child_a_10->id(), 1.0, "" );
  auto child_a_20 = child_o_20->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, true, 0, NodeData::NodeType::ACTION) );
  graph._addNode( child_a_20 );
  graph._addEdge( child_a_20->id(), child_o_20->id(), 1.0, "" );

  graph.saveGraphToFile( "CorrectDecisionEvenWithInfiniteReward.gv" );

  Rewards rewards;
  rewards.setR0(-1);

  auto rewardSetter = [&] ( uint from, uint to, double r )
  {
    rewards.set(from * graph.size() + to, r);
  };

  rewardSetter( 6, 7, -1000.0 );
//  /////
  auto values = ValueIterationAlgorithm::process( graph, rewards );
  auto decided = DecideOnGraphAlgorithm::process( graph, values, rewards );
//  /////

  decided.saveGraphToFile( "CorrectDecisionEvenWithInfiniteRewardDecided.gv" );

  EXPECT_EQ( decided.terminalNodes().size(), 1 );
  EXPECT_EQ( decided.terminalNodes().back().lock()->id(), 4 );
}

TEST(ValueIteration, DecideAfterObservationBranchingPoint2DisymmetricBranchesOneActionStepBefore) {
  LogicEngine engine;
  std::vector< std::string > startStates;
  std::vector< double > egoBeliefState;
  DecisionGraph graph( engine, startStates, egoBeliefState );

  auto root = graph.root();

  // first action + observation
  auto child_o_0 = root->makeChild( DecisionGraph::GraphNodeDataType({"action_0"}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION) );
  graph._addNode( child_o_0 );
  graph._addEdge( child_o_0->id(), root->id(), 1.0, "" );
  auto child_a_00 = child_o_0->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, false, 0, NodeData::NodeType::ACTION) );
  graph._addNode( child_a_00 );
  graph._addEdge( child_a_00->id(), child_o_0->id(), 1.0, "" );
  auto child_o_00 = child_a_00->makeChild( DecisionGraph::GraphNodeDataType({"action_0"}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION) );
  graph._addNode( child_o_00 );
  graph._addEdge( child_o_00->id(), child_a_00->id(), 1.0, "" );

  auto child_o_000 = child_a_00->makeChild( DecisionGraph::GraphNodeDataType({"action_1"}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION) );
  graph._addNode( child_o_000 );
  graph._addEdge( child_o_000->id(), child_a_00->id(), 1.0, "" );
  auto child_a_000 = child_o_000->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, true, 0, NodeData::NodeType::ACTION) );
  graph._addNode( child_a_000 );
  graph._addEdge( child_a_000->id(), child_o_000->id(), 1.0, "" );

  // branch 1 - obs 1
  auto child_a_0 = child_o_00->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, false, 0, NodeData::NodeType::ACTION) );
  graph._addNode( child_a_0 );
  graph._addEdge( child_a_0->id(), child_o_00->id(), 0.5, "" );
  auto child_o_1 = child_a_0->makeChild( DecisionGraph::GraphNodeDataType({"action_1"}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION) );
  graph._addNode( child_o_1 );
  graph._addEdge( child_o_1->id(), root->id(), 1.0, "" );
  auto child_a_f0 = child_o_1->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, true, 0, NodeData::NodeType::ACTION) );
  graph._addNode( child_a_f0 );
  graph._addEdge( child_a_f0->id(), child_o_1->id(), 1.0, "" );

  // branch 2 - obs 2
  auto child_a_10 = child_o_00->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, false, 0, NodeData::NodeType::ACTION) );
  graph._addNode( child_a_10 );
  graph._addEdge( child_a_10->id(), child_o_00->id(), 0.5, "" );
  auto child_o_10 = child_a_10->makeChild( DecisionGraph::GraphNodeDataType({"action_2"}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION) );
  graph._addNode( child_o_10 );
  graph._addEdge( child_o_10->id(), root->id(), 1.0, "" );
  auto child_a_f1 = child_o_10->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, true, 0, NodeData::NodeType::ACTION) );
  graph._addNode( child_a_f1 );
  graph._addEdge( child_a_f1->id(), child_o_10->id(), 1.0, "" );

  graph.saveGraphToFile( "DecideAfterObservationBranchingPoint2DisymmetricBranchesOneActionStepBefore.gv" );

  Rewards rewards;
  rewards.setR0(-1);

  auto rewardSetter = [&] ( uint from, uint to, double r )
  {
    rewards.set(from * graph.size() + to, r);
  };

  rewardSetter( 7, 8, -1.0 );
  rewardSetter( 2, 4, -100.0 );

//  /////
  auto values = ValueIterationAlgorithm::process( graph, rewards );
  auto decided = DecideOnGraphAlgorithm::process( graph, values, rewards );
//  /////

  decided.saveGraphToFile( "DecidedDecideAfterObservationBranchingPoint2DisymmetricBranchesOneActionStepBefore.gv" );

  EXPECT_NEAR( values[0], -3.0, 0.01 );
  EXPECT_NEAR( values[1], -2.0, 0.01 );
  EXPECT_NEAR( values[2], -2.0, 0.01 );
  EXPECT_NEAR( values[3], -1.0, 0.01 );
  EXPECT_NEAR( values[6], -1.0, 0.01 );
  EXPECT_NEAR( values[7], 0.0, 0.01 );

  EXPECT_EQ( decided.terminalNodes().size(), 2 );
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
