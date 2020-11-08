#include <logic_parser.h>

#include <gtest/gtest.h>

using namespace matp;

class ParserTest : public ::testing::Test {
 protected:
  LogicParser w;
};

// Worlds
TEST_F(ParserTest, ThrowIfFileNotFound) {
  ASSERT_THROW( w.parse( "data/nofile.g" ), FolFileNotFound );
}

// Agent Number
TEST_F(ParserTest, AgentNumberSingleAgent1W) {
  w.parse( "data/LGP-overtaking-single-agent-1w.g" );
  ASSERT_EQ( w.agentNumber(), 1 );
}

TEST_F(ParserTest, AgentNumberDoubleAgent1W) {
  w.parse( "data/LGP-overtaking-double-agent-1w.g" );
  ASSERT_EQ( w.agentNumber(), 2 );
}

TEST_F(ParserTest, AgentNumberSingleAgent2W) {
  w.parse( "data/LGP-overtaking-single-agent-2w.g" );
  ASSERT_EQ( w.agentNumber(), 1 );
}

TEST_F(ParserTest, AgentNumberDoubleAgent2W) {
  w.parse( "data/LGP-overtaking-double-agent-2w.g" );
  ASSERT_EQ( w.agentNumber(), 2 );
}

// Total Number Of Worlds
TEST_F(ParserTest, StartStateNumberSingleAgent1W) {
  w.parse( "data/LGP-overtaking-single-agent-1w.g" );
  ASSERT_EQ( w.possibleStartStates().size(), 1 );
}

TEST_F(ParserTest, StartStateNumberSingleAgent2W) {
  w.parse( "data/LGP-overtaking-single-agent-2w.g" );
  ASSERT_EQ( w.possibleStartStates().size(), 2 );
}

// Start state content
TEST_F(ParserTest, StartStatesContentSizeSingleAgent1W) {
  w.parse( "data/LGP-overtaking-single-agent-1w.g" );
  ASSERT_GT( w.possibleStartStates()[ 0 ].size(), 1 );
}

TEST_F(ParserTest, StartStatesContentSizeSingleAgent2W) {
  w.parse( "data/LGP-overtaking-single-agent-2w.g" );
  ASSERT_GT( w.possibleStartStates()[ 0 ].size(), 1 );
  ASSERT_GT( w.possibleStartStates()[ 1 ].size(), 1 );
}

// Ego Agent belief state
TEST_F(ParserTest, BeliefStateSizeSingleAgent1W) {
  w.parse( "data/LGP-overtaking-single-agent-1w.g" );
  ASSERT_EQ( w.egoBeliefState().size(), 1 );
}

TEST_F(ParserTest, BeliefStateSizeSingleAgent2W) {
  w.parse( "data/LGP-overtaking-single-agent-2w.g" );
  ASSERT_EQ( w.egoBeliefState().size(), 2 );
}

TEST_F(ParserTest, BeliefStateSizeDoubleAgent1W) {
  w.parse( "data/LGP-overtaking-double-agent-1w.g" );
  ASSERT_EQ( w.egoBeliefState().size(), 1 );
}

TEST_F(ParserTest, BeliefStateSizeDoubleAgent2W) {
  w.parse( "data/LGP-overtaking-double-agent-2w.g" );
  ASSERT_EQ( w.egoBeliefState().size(), 2 );
}

// Ill defined files
TEST_F(ParserTest, WrongBeliefStateSize) {
  ASSERT_THROW( w.parse( "data/LGP-overtaking-single-agent-2w-wrong-belief-state-size.g" ), IncoherentDefinition );
}

TEST_F(ParserTest, WrongBeliefProbabilities) {
  ASSERT_THROW( w.parse( "data/LGP-overtaking-single-agent-2w-wrong-probabilities.g" ), IncoherentDefinition );
}

// Agent Actions Number
TEST_F(ParserTest, FirstAgentActionsNumber) {
  w.parse( "data/LGP-overtaking-single-agent-1w.g" );
  ASSERT_EQ( w.totalActionsNumber( 0 ), 3 );
}

TEST_F(ParserTest, SecondAgentActionsNumber) {
  w.parse( "data/LGP-overtaking-double-agent-1w.g" );
  ASSERT_EQ( w.totalActionsNumber( 1 ), 3 );
}

TEST_F(ParserTest, ThrowIfActionsWithoutPrefix) {
  ASSERT_THROW( w.parse( "data/LGP-overtaking-double-agent-2w-wrong-action-definitions.g" ), IncoherentDefinition );
}

TEST_F(ParserTest, LogicEngineInitialized) {
  w.parse( "data/LGP-overtaking-double-agent-2w.g" );
  ASSERT_TRUE( w.engine().initialized() );
}

TEST_F(ParserTest, ReapplyStartState) {
  w.parse( "data/LGP-overtaking-single-agent-1w.g" );
  auto engine = w.engine();
  auto startStates = w.possibleStartStates();
  auto bs = w.egoBeliefState();

  engine.resetState();
  auto n = engine.getPossibleActions( 0 ).size();
  auto startState = engine.getState();
  engine.setState( startStates.back() );
  auto nn = engine.getPossibleActions( 0 ).size();

  ASSERT_EQ( n, nn );
}


//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
