#pragma once

#include <set>
#include <queue>
#include <vector>
#include <string>
#include <unordered_map>

#include <logic_engine.h>

#include <graph_node.h>

namespace matp
{

class Rewards
{
public:
  void setR0(double r0) { r0_ = r0; }

  double get(uint key)
  {
    if(rewards_.find(key) == rewards_.end())
      rewards_[key] = r0_;

    return rewards_[key];
  }

  void set(uint key, double r)
  {
    rewards_[key] = r;
  }

private:
  std::unordered_map< uint, double > rewards_;
  double r0_{-1.0};
};

struct NodeData
{
  enum class NodeType
  {
    ACTION = 0, // an action has to be taken at this node
    OBSERVATION
  };

  NodeData()
    : states()
    , beliefState()
    //, leadingArtifact()
    , terminal( false )
    , agentId( 0 )
    , nodeType( NodeType::ACTION )
  {
    computeHash();
  }

  NodeData( const std::vector< std::string > & states,
            const std::vector< double      > & beliefState,
            //const std::string leadingArtifact,
            bool terminal,
            uint agentId,
            NodeType nodeType )
    : states( states )
    , beliefState( beliefState )
    //, leadingArtifact( leadingArtifact )
    , terminal( terminal )
    , agentId( agentId )
    , nodeType( nodeType )
  {
    computeHash();
  }

  std::vector< std::string > states;
  std::vector< double      > beliefState;
  //std::string leadingArtifact; // leading action of leading observation
  //
  bool terminal;
  uint agentId;
  NodeType nodeType;

  std::size_t hash() const
  {
    return hash_;
  }

private:
  void computeHash()  // element depending on the parent are not included into the hash
  {
    hash_ = 0;

    CHECK_EQ( states.size(), beliefState.size(), "corrrupted belief state" );

    for( auto i = 0; i < states.size(); ++i )
    {
      auto s = states[i];
      auto p = beliefState[i];
      if( p > 10e-8 )
      {
        hash_+=std::hash<std::string>()(s);
        hash_+=std::hash<double>()(p);
      }
    }
    hash_+=std::hash<bool>()(terminal);
    hash_+=std::hash<uint>()(agentId);
    hash_+=std::hash<uint>()((uint)nodeType);
  }

  std::size_t hash_;
};

bool sameState ( const NodeData & a, const NodeData & b );

std::ostream& operator<<(std::ostream& stream, NodeData const& data);

class DecisionGraph
{
public:
  using GraphNodeDataType = NodeData;
  using GraphNodeType = GraphNode< NodeData >;
  using EdgeDataType = std::unordered_map< uint, std::pair< double, std::string > >;

public:
  DecisionGraph() = default;
  void reset();

  DecisionGraph( const DecisionGraph & ); // copy ctor
  DecisionGraph& operator= ( const DecisionGraph & ); // assignment operator

  DecisionGraph( const LogicEngine &, const std::vector< std::string > & startStates, const std::vector< double > & egoBeliefState );
  bool empty() const { return nodes_.size() <= 1; } // root node
  std::size_t size() const { return nodes_.size(); }
  void build( int maxSteps, bool graph = false );
  std::queue< GraphNodeType::ptr > expand( const GraphNodeType::ptr & node );
  GraphNodeType::ptr root() const { return root_; }
  std::vector< std::weak_ptr< GraphNodeType > > nodes() const { return nodes_; }
  std::list< std::weak_ptr< GraphNodeType > > terminalNodes() const { return terminalNodes_; }
  std::vector< EdgeDataType > edges() const { return edges_; }

  void _addNode( const std::weak_ptr< GraphNodeType > & node ); // for tests only!!
  void _addEdge( uint child, uint parent, double p, const std::string & artifact ); // for tests only!!

  void removeNode( const std::weak_ptr< GraphNodeType > & node );
  void purgeNodes( const std::vector< bool > & ); // remove nodes from nodes_ and terminalNodes_ that are not valid anymore
  void saveGraphToFile( const std::string & filename ) const;

  // public for testing purpose
  std::vector< std::string > getCommonPossibleActions( const GraphNodeType::ptr & node, uint agentId ) const;
  std::vector< std::tuple< double, NodeData, std::string > > getPossibleOutcomes( const GraphNodeType::ptr & node, const std::string & action ) const;

private:
  void copy( const DecisionGraph & );

private:
  mutable LogicEngine engine_;
  // nodes
  GraphNodeType::ptr root_;
  std::vector< std::weak_ptr< GraphNodeType > > nodes_;
  std::unordered_map< std::size_t, std::list< uint > > hash_to_id_;
  std::list< std::weak_ptr< GraphNodeType > > terminalNodes_;
  bool isGraph_ = false; // if false is only a tree
  // edges
  std::vector< EdgeDataType > edges_;// child-id -> <p, leading artifact> p = // probability to reach this node given the parent, usage edges_[child_id] -> gives a map of from->p
};
} // namespace matp
