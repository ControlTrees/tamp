#include <boost/algorithm/string/replace.hpp>

#include <logic_engine.h>

#include <gtest/gtest.h>

using namespace matp;

// LogicEngine
TEST(LogicEngine, NoInitializedIfDefaultConstructed) {
  LogicEngine engine;
  ASSERT_FALSE( engine.initialized() );
}

// LogicEngine
TEST(LogicEngine, InitializedIfConstructedWithFile) {
  LogicEngine engine( "data/LGP-overtaking-single-agent-1w.g" );
  ASSERT_TRUE( engine.initialized() );
}

TEST(LogicEngine, NPossibleActionsSingleAgent) {
  LogicEngine engine( "data/LGP-overtaking-single-agent-1w.g" );
  ASSERT_EQ( engine.getPossibleActions( 0 ).size(), 2 );
}

TEST(LogicEngine, NPossibleActionsDoubleAgent1) {
  LogicEngine engine( "data/LGP-overtaking-double-agent-1w.g" );
  ASSERT_EQ( engine.getPossibleActions( 0 ).size(), 2 );
}

TEST(LogicEngine, NPossibleActionsDoubleAgent2) {
  LogicEngine engine( "data/LGP-overtaking-double-agent-1w.g" );
  ASSERT_EQ( engine.getPossibleActions( 1 ).size(), 3 );
}

TEST(LogicEngine, AgentIdOutOfBound) {
  LogicEngine engine( "data/LGP-overtaking-double-agent-1w.g" );
  ASSERT_TRUE( engine.getPossibleActions( 2 ).empty() );
}

TEST(LogicEngine, EngineCopyable) {
  LogicEngine engine( "data/LGP-overtaking-double-agent-1w.g" );
  LogicEngine copy ( engine );
  ASSERT_EQ( copy.getPossibleActions( 1 ).size(), 3 );
}

TEST(LogicEngine, EngineAssignable) {
  LogicEngine engine( "data/LGP-overtaking-double-agent-1w.g" );
  LogicEngine copy;
  copy = engine;
  ASSERT_EQ( copy.getPossibleActions( 1 ).size(), 3 );
}

// Actions
TEST(LogicEngine, SingleAgentApplyLook) {
  LogicEngine engine( "data/LGP-overtaking-single-agent-1w.g" );
  auto actions = engine.getPossibleActions( 0 );
  engine.transition( actions[0] );
  auto state = engine.getState();
  ASSERT_TRUE( state.find( "(observed lanes)" ) != -1 );
}

TEST(LogicEngine, SingleAgentApplyFollow) {
  LogicEngine engine( "data/LGP-overtaking-single-agent-1w.g" );
  auto actions = engine.getPossibleActions( 0 );
  engine.transition( actions[1] );
  auto state = engine.getState();
  ASSERT_TRUE( state.find( "(following)" ) != -1 );
}

TEST(LogicEngine, DoubleAgentApplyAccelerating) {
  LogicEngine engine( "data/LGP-overtaking-double-agent-1w.g" );
  auto actions = engine.getPossibleActions( 1 );
  engine.transition( actions[0] );
  auto state = engine.getState();
  ASSERT_TRUE( state.find( "(accelerating agent_1)" ) != -1 );
}

TEST(LogicEngine, DoubleAgentApplyAccelerating2W) {
  LogicEngine engine( "data/LGP-overtaking-double-agent-2w.g" );
  auto actions = engine.getPossibleActions( 1 );
  engine.transition( actions[0] );
  auto state = engine.getState();
  ASSERT_TRUE( state.find( "(accelerating agent_1)" ) != -1 );
}

// Get and reset states
TEST(LogicEngine, SingleAgentSetState) {
  LogicEngine engine( "data/LGP-overtaking-single-agent-2w.g" );
  auto initState = engine.getState();
  auto actions = engine.getPossibleActions( 0 );
  engine.transition( actions[1] );
  engine.setState( initState );
  auto state = engine.getState();

  //boost::replace_all(initState, ": ", ":");
  std::cout << state << " " << initState  << std::endl;

  ASSERT_TRUE( state == initState );
}


TEST(LogicEngine, DoubleAgentSetState) {
  LogicEngine engine( "data/LGP-overtaking-double-agent-2w.g" );
  auto initState = engine.getState();
  auto actions = engine.getPossibleActions( 1 );
  engine.transition( actions[0] );
  engine.setState( initState );
  auto state = engine.getState();

  //boost::replace_all(initState, ": ", ":");
  //boost::replace_all(state, ": ", ":");

  std::cout << state << " " << initState  << std::endl;

  ASSERT_TRUE( state == initState );
}

TEST(LogicEngine, DoubleAgentSetState2) {
  LogicEngine engine( "data/LGP-overtaking-double-agent-2w.g" );

  auto initState = engine.getState();
  auto actions = engine.getPossibleActions( 1 );
  engine.setState( initState );
  auto actionss = engine.getPossibleActions( 1 );

  ASSERT_EQ( actions.size(), actionss.size() );
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
