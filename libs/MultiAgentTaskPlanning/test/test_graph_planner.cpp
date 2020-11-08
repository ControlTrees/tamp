#include <graph_planner.h>

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

using namespace matp;

class GraphPlannerTest : public ::testing::Test {
 protected:
  GraphPlanner tp;
};

// GraphPlanner
TEST_F(GraphPlannerTest, ThrowIfFileNotFound) {
  ASSERT_THROW( tp.setFol( "data/nofile.g" ), FolFileNotFound );
}

TEST_F(GraphPlannerTest, ThrowIfNoAgentFiles) {
  ASSERT_THROW( tp.setFol( "" ), FolFileNotFound );
}

TEST_F(GraphPlannerTest, AgentNumber) {
  tp.setFol( "data/LGP-overtaking-double-agent-2w.g" );

  ASSERT_EQ( tp.agentNumber(), 2 );
}

TEST_F(GraphPlannerTest, emptyGraph) {
  tp.buildGraph();

  auto graph = tp.decisionGraph();

  ASSERT_TRUE( graph.empty() );
}

TEST_F(GraphPlannerTest, OneNodeIfCorrectFile) {
  tp.setFol( "data/LGP-overtaking-single-agent-1w.g" );

  auto graph = tp.decisionGraph();

  ASSERT_TRUE( graph.size() == 1 );
}

TEST_F(GraphPlannerTest, buildGraph) {
  tp.setFol( "data/LGP-overtaking-single-agent-2w.g" );
  tp.setMaxDepth( 3 );

  tp.buildGraph();

  tp.saveGraphToFile( "LGP-overtaking-single-agent-2w.gv" );
  auto graph = tp.decisionGraph();
  ASSERT_TRUE( graph.size() > 1 );
}

TEST_F(GraphPlannerTest, buildGraphDoubleAgent1w) {
  tp.setFol( "data/LGP-overtaking-double-agent-1w.g" );
  tp.setMaxDepth( 2 );

  tp.buildGraph();

  tp.saveGraphToFile( "LGP-overtaking-double-agent-1w.gv" );
  auto graph = tp.decisionGraph();
  ASSERT_TRUE( graph.size() > 1 );
}

TEST_F(GraphPlannerTest, buildBlocksSingleAgent1w) {
  tp.setFol( "data/LGP-blocks-fol-1w-model-2-unified.g" );
  tp.setMaxDepth( 20 );

  tp.buildGraph( true );

  auto graph = tp.decisionGraph();
  graph.saveGraphToFile( "LGP-blocks-fol-1w-model-2-unified.gv" );

  ASSERT_TRUE( graph.size() > 1 );
}

//TEST_F(GraphPlannerTest, solveDoubleAgent1w) {
//  tp.setFol( "data/LGP-overtaking-double-agent-1w.g" );
//  tp.solve();
//  auto policy = tp.getPolicy();
//  ASSERT_NE( policy, nullptr );
//}

TEST_F(GraphPlannerTest, solveDoubleAgent2w) {
  tp.setFol( "data/LGP-overtaking-double-agent-2w.g" );

  tp.solve();

  auto policy = tp.getPolicy();
  ASSERT_FALSE( policy.empty() );
}

TEST_F(GraphPlannerTest, DecisionArtifactToKomoTag) {
  auto leadingArtifact = "(actionName X Y Z)";
  auto args = decisionArtifactToKomoArgs( leadingArtifact );

  ASSERT_EQ( args[0]    , "actionName" );
  ASSERT_EQ( args[1]    , "X" );
  ASSERT_EQ( args[2]    , "Y" );
  ASSERT_EQ( args[3]    , "Z" );
}

TEST_F(GraphPlannerTest, DecisionGraphNodeToPolicyNode) {
  auto r0 = -2;
  auto agentId = 5;
  //auto beliefState = std::vector< double >( { 0.4, 0.6 } );
  auto leadingArtifact = "(actionName X Y Z)";
  auto nodeType = NodeData::NodeType::ACTION;
  auto p = 0.5;
  auto states = std::vector< std::string >( { "", "" } );
  auto terminal = false;

  tp.setR0( r0 );

  NodeData nData;
  nData.agentId = agentId;
  //nData.beliefState = beliefState;
  //nData.leadingArtifact = leadingArtifact;
  nData.nodeType = nodeType;
  //nData.p = p;
  nData.states = states;
  nData.terminal = terminal;

  auto pData = tp.decisionGraphtoPolicyData( nData, 1 );

  //ASSERT_EQ( pData.beliefState    , beliefState );
  ASSERT_EQ( pData.markovianReturn, r0 );
  ASSERT_EQ( pData.decisionGraphNodeId, 1 );
}

TEST_F(GraphPlannerTest, PolicySaveSingleAgent2W) {
  tp.setFol( "data/LGP-overtaking-single-agent-2w.g" );

  tp.solve();

  auto policy = tp.getPolicy();
  const std::string policyFileName( "LGP-overtaking-single-agent-2w" );
  policy.save( policyFileName + ".po" );
  policy.saveToGraphFile( policyFileName + ".gv" );
  ASSERT_TRUE( boost::filesystem::exists( policyFileName + ".po" ) );
  ASSERT_TRUE( boost::filesystem::exists( policyFileName + ".gv" ) );
}

TEST_F(GraphPlannerTest, PolicySaveDoubleAgent1W) {
  tp.setFol( "data/LGP-overtaking-double-agent-1w.g" );
  tp.setMaxDepth( 2 );

  tp.solve();

  auto policy = tp.getPolicy();
  const std::string policyFileName( "LGP-overtaking-double-agent-1w" );
  policy.save( policyFileName + ".po" );
  policy.saveToGraphFile( policyFileName + ".gv" );
  tp.saveGraphToFile(policyFileName + "-full.gv");
  tp.saveDecidedGraphToFile(policyFileName + "-decided.gv");
  ASSERT_TRUE( boost::filesystem::exists( policyFileName + ".po" ) );
  ASSERT_TRUE( boost::filesystem::exists( policyFileName + ".gv" ) );
  ASSERT_TRUE( boost::filesystem::exists( policyFileName + ".png" ) );
}

TEST_F(GraphPlannerTest, PolicySaveDoubleAgent1WTweaked) {
  tp.setFol( "data/LGP-overtaking-double-agent-1w.g" );
  tp.setMaxDepth( 2 );

  tp.solve();

  auto policy = tp.getPolicy();
  const std::string policyFileName( "LGP-overtaking-double-agent-1w-tweaked" );
  auto leafs = policy.leafs();
  leafs.front().lock()->data().leadingKomoArgs = { "__AGENT_0__follow", "truck" };
  policy.save( policyFileName + ".po" );
  policy.saveToGraphFile( policyFileName + ".gv" );
  tp.saveDecidedGraphToFile(policyFileName + "-decided.gv");
  ASSERT_TRUE( boost::filesystem::exists( policyFileName + ".po" ) );
  ASSERT_TRUE( boost::filesystem::exists( policyFileName + ".gv" ) );
  ASSERT_TRUE( boost::filesystem::exists( policyFileName + ".png" ) );
}

TEST_F(GraphPlannerTest, PolicySaveDoubleAgent2W) {
  tp.setFol( "data/LGP-overtaking-double-agent-2w.g" );
  tp.setMaxDepth( 2 );

  tp.solve();

  auto policy = tp.getPolicy();
  const std::string policyFileName( "LGP-overtaking-double-agent-2w" );
  tp.saveDecidedGraphToFile(policyFileName + "-decided.gv");
  policy.save( policyFileName + ".po" );
  policy.saveToGraphFile( policyFileName + ".gv" );
  ASSERT_TRUE( boost::filesystem::exists( policyFileName + ".po" ) );
  ASSERT_TRUE( boost::filesystem::exists( policyFileName + ".gv" ) );
}

TEST_F(GraphPlannerTest, PolicyLeafs) {
  tp.setFol( "data/LGP-overtaking-single-agent-1w.g" );
  tp.setMaxDepth( 2 );

  tp.solve();

  tp.saveGraphToFile( "LGP-overtaking-single-agent-1w-decision-graph.gv" );
  tp.saveDecidedGraphToFile( "LGP-overtaking-single-agent-1w-decided-decision-graph.gv" );
  auto policy = tp.getPolicy();
  const std::string policyFileName( "LGP-overtaking-single-agent-1w" );
  policy.save( policyFileName + ".po" );
  policy.saveToGraphFile( policyFileName + ".gv" );
  auto leafs = policy.leafs();
  auto leaf = leafs.front();
  EXPECT_EQ( leafs.size(), 1 );
  EXPECT_EQ( leaf.lock()->id(), 2 );
}

TEST_F(GraphPlannerTest, PolicyValue)
{
  tp.setFol( "data/LGP-overtaking-single-agent-1w.g" );
  tp.setMaxDepth( 2 );

  tp.solve();

  auto policy = tp.getPolicy();
  EXPECT_NEAR( policy.value(), -2.0, 0.01);
}

TEST_F(GraphPlannerTest, IntegratePolicy)
{
  tp.setFol( "data/LGP-overtaking-single-agent-1w.g" );
  tp.setMaxDepth( 2 );
  tp.solve();
  auto policy = tp.getPolicy();
  policy.root()->data().markovianReturn = 0.0;
  auto child = policy.root()->children().front();
  child->data().status = PolicyNodeData::INFORMED;
  child->data().markovianReturn = -0.3;

  tp.integrate( policy );

  EXPECT_EQ( tp.reward( policy.root()->id(), child->id() ), -0.3 );
  //EXPECT_EQ( tp.reward( child->id() ), -0.3 );
}

TEST_F(GraphPlannerTest, IntegratePolicy_OnlyReturnOfPlannedNodeIsIntegrated)
{
  tp.setFol( "data/LGP-overtaking-single-agent-1w.g" );
  tp.setMaxDepth( 2 );
  tp.solve();
  auto policy = tp.getPolicy();
  policy.root()->data().markovianReturn = 0.0;
  auto child = policy.root()->children().front();
  child->data().status = PolicyNodeData::UNPLANNED;
  child->data().markovianReturn = -0.3;

  tp.integrate( policy );

  EXPECT_EQ( tp.reward( policy.root()->id(), child->id() ), tp.R0() );
  //EXPECT_EQ( tp.reward( child->id() ), -0.3 );
}

TEST_F(GraphPlannerTest, ValueIteration) {
  tp.setFol( "data/LGP-overtaking-single-agent-1w.g" );
  tp.setMaxDepth( 2 );

  tp.solve();

  auto values = tp.values();

  const auto eps = 10e-3;
  const auto inf = -10e8;

  EXPECT_NEAR(values[0], -2, eps);

  EXPECT_NEAR(values[1], -1, eps);
  EXPECT_NEAR(values[2], -1, eps);

  EXPECT_NEAR(values[5], 0, eps);
  EXPECT_NEAR(values[6], 0, eps);

  EXPECT_LE(values[3], inf);
  EXPECT_LE(values[4], inf);

  EXPECT_LE(values[7], inf);
  EXPECT_LE(values[8], inf);

  EXPECT_LE(values[9], inf);
  EXPECT_LE(values[10], inf);
}

TEST_F(GraphPlannerTest, solveGraph2W) {
  tp.setFol( "data/LGP-overtaking-single-agent-2w.g" );
  tp.setMaxDepth( 10 );

  tp.buildGraph( true );
  tp.solve();

  auto policy = tp.getPolicy();

  std::string policyFileName = "LGP-overtaking-single-agent-2w";
  policy.save( policyFileName + ".po" );
  policy.saveToGraphFile( policyFileName + ".gv" );
}

TEST_F(GraphPlannerTest, compareNumberOfNodesOfDecidedGraphAndPolicy) {
  tp.setFol( "data/LGP-overtaking-single-agent-2w.g" );
  tp.setMaxDepth( 10 );

  tp.buildGraph( true );
  tp.solve();

  auto decided = tp.decidedDecisionGraph();
  auto policy = tp.getPolicy();

  std::string decidedFileName = "LGP-overtaking-single-agent-2w-graph-decided";
  decided.saveGraphToFile( decidedFileName + ".gv" );

  EXPECT_EQ( decided.size(), 8 );
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
