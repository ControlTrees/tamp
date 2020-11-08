#pragma once

#include <fstream>
#include <po_graph_node.h>

namespace tp
{

class GraphPrinter
{
public:
  GraphPrinter( std::ostream & ss )
    : ss_( ss )
  {

  }

  void print( const POGraphNode::ptr & node, std::list< POGraphNode::ptr > terminals = std::list< POGraphNode::ptr >() );

private:
  void saveGraphFrom( const POGraphNode::ptr & node );
  std::string saveObservationNode( const POGraphNode::ptr & a, const POGraphNode::ptr & b );
  void saveActionEdge( const POGraphNode::ptr & a, const POGraphNode::ptr & b );
  void saveObservationEdge( const std::string & oid, const POGraphNode::ptr & a, const POGraphNode::ptr & b );

private:
  std::ostream & ss_;
  std::list< std::pair< POGraphNode::ptr, POGraphNode::ptr > > savedEdges_;
};

}
