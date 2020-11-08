#include <decision_graph.h>
#include <logic_parser.h>

#include <gtest/gtest.h>

using namespace matp;

// DecisionGraph
TEST(DecisionGraph, DefaultIsEmpty) {
  DecisionGraph graph;
  ASSERT_EQ( graph.empty(), true );
}

// Actions
TEST(DecisionGraph, getCommonPossibleActions1W) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-1w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  ASSERT_EQ( graph.getCommonPossibleActions( graph.root(), 0 ).size(), p.engine().getPossibleActions( 0 ).size() );
}

TEST(DecisionGraph, getCommonPossibleActions2W) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" ); // one action not possible because we don't know if the lane is free or not
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  ASSERT_EQ( graph.getCommonPossibleActions( graph.root(), 0 ).size(), 2 );
}

TEST(DecisionGraph, NoCommonActions) {
  LogicParser p;
  p.parse( "data/LGP-single-agent-2w-no-common-actions.g" ); // one action not possible because we don't know if the lane is free or not
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );

  ASSERT_EQ( graph.getCommonPossibleActions( graph.root(), 0 ).size(), 0 );
}

// Apply states PO
TEST(DecisionGraph, applyPOStates) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  auto e = p.engine();
  auto startState = graph.root()->data().states[ 1 ];
  ASSERT_NE( startState, e.getState() );
  e.setState( startState );
  ASSERT_EQ( startState, e.getState() );
}

// Outcomes
TEST(DecisionGraph, getPossibleOutcomes1W) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-1w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  auto actions = graph.getCommonPossibleActions( graph.root(), 0 );
  ASSERT_EQ( graph.getPossibleOutcomes( graph.root(), actions[ 1 ] ).size(), 1 );
}

TEST(DecisionGraph, getPossibleOutcomes2W) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  auto actions = graph.getCommonPossibleActions( graph.root(), 0 );
  ASSERT_EQ( graph.getPossibleOutcomes( graph.root(), actions[ 0 ] ).size(), 2 );
}

// Expansion
TEST(DecisionGraph, expandFromRoot2W) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  auto actions = graph.getCommonPossibleActions( graph.root(), 0 );
  graph.expand( graph.root() );
  auto rootChildren = graph.root()->children();
  ASSERT_EQ( graph.size(), 6 );
  ASSERT_EQ( rootChildren.size(), actions.size() );
}

TEST(DecisionGraph, expandCheckStateInequality) {  // graph connection if an equivalent symbolic state exits
  {
  DecisionGraph::GraphNodeDataType data1( {"vdvf"}, {1.0}, false, 0, NodeData::NodeType::ACTION );
  DecisionGraph::GraphNodeDataType data2( {"vdvfe"}, {1.0}, false, 0, NodeData::NodeType::ACTION );

  EXPECT_FALSE( sameState( data1, data2 ) );
  }
  {
  DecisionGraph::GraphNodeDataType data1( {"vdvf"}, {1.0}, false, 0, NodeData::NodeType::ACTION );
  DecisionGraph::GraphNodeDataType data2( {"vdvf"}, {0.99}, false, 0, NodeData::NodeType::ACTION );

  EXPECT_FALSE( sameState( data1, data2 ) );
  }
  {
  DecisionGraph::GraphNodeDataType data1( {"vdvf"}, {1.0}, false, 0, NodeData::NodeType::ACTION );
  DecisionGraph::GraphNodeDataType data2( {"vdvf"}, {1.0}, true, 0, NodeData::NodeType::ACTION );

  EXPECT_FALSE( sameState( data1, data2 ) );
  }
  {
  DecisionGraph::GraphNodeDataType data1( {"vdvf"}, {1.0}, false, 0, NodeData::NodeType::ACTION );
  DecisionGraph::GraphNodeDataType data2( {"vdvf"}, {1.0}, false, 1, NodeData::NodeType::ACTION );

  EXPECT_FALSE( sameState( data1, data2 ) );
  }
  {
  DecisionGraph::GraphNodeDataType data1( {"vdvf"}, {1.0}, false, 0, NodeData::NodeType::ACTION );
  DecisionGraph::GraphNodeDataType data2( {"vdvf"}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION );

  EXPECT_FALSE( sameState( data1, data2 ) );
  }
}
TEST(DecisionGraph, expandCheckStateEquality) {  // graph connection if an equivalent symbolic state exits
  {
  DecisionGraph::GraphNodeDataType data1( {"vdvf"}, {1.0}, false, 0, NodeData::NodeType::ACTION );
  DecisionGraph::GraphNodeDataType data2( {"vdvf"}, {1.0}, false, 0, NodeData::NodeType::ACTION );

  ASSERT_TRUE( sameState( data1, data2 ) );
  }

  {
  DecisionGraph::GraphNodeDataType data1( {"vdvf"}, {1.0}, false, 0, NodeData::NodeType::ACTION );
  DecisionGraph::GraphNodeDataType data2( {"vdvf"}, {1.0}, false, 0, NodeData::NodeType::ACTION );

  ASSERT_TRUE( sameState( data1, data2 ) );
  }
}

TEST(DecisionGraph, expandCheckStateEqualityAfterExpansion) {  // graph connection if an equivalent symbolic state exits
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  auto actions = graph.getCommonPossibleActions( graph.root(), 0 );
  graph.expand( graph.root() );
  graph.saveGraphToFile( "expandRewires-1.gv" );

  auto rootChildren = graph.root()->children();
  ASSERT_EQ( graph.size(), 6 );

  // expand 5
  for( auto c : graph.nodes() )
  {
    auto cc = c.lock();
    if( cc->id() == 5 )
    {
      graph.expand( cc );
      graph.saveGraphToFile( "expandRewires-2.gv" );
    }
  }
  std::map< uint, DecisionGraph::GraphNodeDataType > data;
  // compare state of 4, 5, 9, 10
  for( auto c : graph.nodes() )
  {
    auto cc = c.lock();
    data[ cc->id() ] = cc->data();
  }

  ASSERT_TRUE( sameState( data[2], data[7] ) );
  ASSERT_TRUE( sameState( data[3], data[8] ) );
  ASSERT_FALSE( sameState( data[2], data[8] ) );
  ASSERT_FALSE( sameState( data[3], data[7] ) );
}

TEST(DecisionGraph, expandFromRootDouble2W) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  auto actions = graph.getCommonPossibleActions( graph.root(), 0 );
  graph.expand( graph.root() );
  auto rootChildren = graph.root()->children();
  ASSERT_EQ( graph.size(), 24 );
  ASSERT_EQ( rootChildren.size(), actions.size() );
}

TEST(DecisionGraph, expandFromRootDouble2WFringeSize) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  auto queue = graph.expand( graph.root() );
  ASSERT_EQ( queue.size(), 9 );
}

// Agent Id
TEST(DecisionGraph, agentId) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.expand( graph.root() );
  auto observationNodes = graph.root()->children();
  auto actionNodes = graph.root()->children().back()->children();

  ASSERT_EQ( observationNodes.back()->data().agentId, 0 );
  ASSERT_EQ( actionNodes.back()->data().agentId, 1 );
}

// Node types
TEST(DecisionGraph, expandedNodesTypes) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.expand( graph.root() );
  for( auto child : graph.root()->children() )
  {
    ASSERT_EQ( child->data().nodeType, NodeData::NodeType::OBSERVATION );

    for( auto childChild : child->children() )
    {
      ASSERT_EQ( childChild->data().nodeType, NodeData::NodeType::ACTION );
    }
  }
}

// Artifacts
TEST(DecisionGraph, expandedLeadingAction) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.expand( graph.root() );

  auto leadingAction = graph.edges() [ 1 ][ 0 ].second;
  ASSERT_NE( leadingAction.find( "look lanes" ), std::string::npos );
}

TEST(DecisionGraph, expandedLeadingObservation0) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.expand( graph.root() );
  auto childChild = graph.root()->children().back()->children().back();

  auto leadingObservation = graph.edges() [ childChild->id() ][ 0 ].second;
  ASSERT_EQ( leadingObservation, "" );
}

TEST(DecisionGraph, expandedLeadingObservation1) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.expand( graph.root() );
  auto childChild = graph.root()->children().front()->children().front();

  auto leadingObservation = graph.edges() [ childChild->id() ][ 1 ].second;
  ASSERT_NE( leadingObservation.find( "free lane_2" ), std::string::npos );
}

TEST(DecisionGraph, expandedLeadingObservation2) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.expand( graph.root() );
  auto childChild = graph.root()->children().front()->children().back();

  auto leadingObservation = graph.edges() [ childChild->id() ][ 0 ].second;;
  ASSERT_EQ( leadingObservation, "" );
}

// Build
TEST(DecisionGraph, buildFromRootDouble2WD3) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(1);
  ASSERT_EQ( graph.size(), 24 );
}

TEST(DecisionGraph, decisionGraphCopy1) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(1);
  auto graphCopy = graph;

  ASSERT_NE( graph.root(), graphCopy.root() );
  ASSERT_EQ( graph.size(), graphCopy.size() );
}

TEST(DecisionGraph, decisionGraphAssignment) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(1);
  DecisionGraph graphCopy;
  graphCopy = graph;

  ASSERT_NE( graph.root(), graphCopy.root() );
  ASSERT_EQ( graph.size(), graphCopy.size() );
}

TEST(DecisionGraph, decisionGraphAssignment2) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-1w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(3);
  DecisionGraph graphCopy;
  graphCopy = graph;

  std::list< std::pair< DecisionGraph::GraphNodeType::ptr, DecisionGraph::GraphNodeType::ptr > > Q;
  Q.push_back( std::make_pair( graph.root(), graphCopy.root() ) );

  while( ! Q.empty() )
  {
    auto pair = Q.front();
    Q.pop_front();

    auto original = pair.first;
    auto copy     = pair.second;

    EXPECT_EQ( original->id(), copy->id() );

    auto originalChildrenL = original->children();
    auto copyChildrenL     = copy->children();
    std::vector< DecisionGraph::GraphNodeType::ptr > originalChildren{ originalChildrenL.begin(), originalChildrenL.end() };
    std::vector< DecisionGraph::GraphNodeType::ptr > copyChildren{ copyChildrenL.begin(), copyChildrenL.end() };

    for( auto i = 0; i < originalChildren.size(); ++i )
    {
      Q.push_back( std::make_pair( originalChildren[i], copyChildren[i] ) );
    }
  }

  ASSERT_NE( graph.root(), graphCopy.root() );
  ASSERT_EQ( graph.size(), graphCopy.size() );
}

TEST(DecisionGraph, buildCheckNodeDepth) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(1);
  for( auto node : graph.nodes() )
  {
    ASSERT_LE( node.lock()->depth(), 4 );
  }
}


// Terminal label
TEST(DecisionGraph, terminalNodesAD) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );

  graph.build(2);

  auto leafs = graph.terminalNodes();
  bool found = false;
  int agentId = -1;
  for( auto weakL : leafs )
  {
    auto l = weakL.lock();

    if( l->id() == 25 )
    {
      found = true;
      agentId = l->data().agentId;
    }
  }

  ASSERT_TRUE( found );
  ASSERT_EQ( agentId, 1 );// terminal a last ego action
}

TEST(DecisionGraph, terminalNodesBLOCKS) {
  LogicParser p;
  p.parse( "data/LGP-blocks-fol-1w-model-2.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );

  graph.build(7);

  auto leafs = graph.terminalNodes();

  ASSERT_TRUE( ! leafs.empty() );
}

// Terminal label
//TEST(DecisionGraph, probabilityAtObservationNode) {
//  LogicParser parser;
//  parser.parse( "data/LGP-overtaking-double-agent-2w.g" );
//  DecisionGraph graph( parser.engine(), parser.possibleStartStates(), parser.egoBeliefState() );
//  graph.build(2);
//  auto nodes = graph.nodes();

//  double p = -1;
//  for( auto l : nodes )
//  {
//    if( l.lock()->id() == 2 )
//    {
//      p = l.lock()->data().p;
//    }
//  }

//  ASSERT_EQ( p, 0.95 );
//}

TEST(DecisionGraph, terminalNodesHaveNoChildren) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(2);
  auto leafs = graph.terminalNodes();

  for( auto weakL : leafs )
  {
    auto l = weakL.lock();

    ASSERT_EQ( l->children().size(), 0 );
  }
}

// Print graph
TEST(DecisionGraph, buildGraphAndPrint2WD1) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(1);
  graph.saveGraphToFile( "buildGraphAndPrint2WD1.gv" );
}

TEST(DecisionGraph, buildGraphAndPrint2WD2) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(2);
  graph.saveGraphToFile( "buildGraphAndPrint2WD2.gv" );
}

TEST(DecisionGraph, buildGraphAndPrint1WD2) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-1w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(2);
  graph.saveGraphToFile( "buildGraphAndPrint1WD2.gv" );
}

// Specific to graphs (vs. tree)
TEST(DecisionGraph, buildGraph2W) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(10, true);
  graph.saveGraphToFile("LGP-overtaking-single-agent-2w-graph.gv");
  ASSERT_EQ( graph.size(), 14 );
}

TEST(DecisionGraph, copyOfGraph) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(10, true);
  DecisionGraph graphCopy( graph );

  EXPECT_EQ( graph.size(), graphCopy.size() );
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
