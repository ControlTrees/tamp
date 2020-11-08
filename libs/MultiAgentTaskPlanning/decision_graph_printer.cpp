#include <decision_graph_printer.h>

#include <boost/algorithm/string/replace.hpp>

namespace matp
{

void GraphPrinter::print( const DecisionGraph & graph )
{
  if( ! graph.root() )
  {
    return;
  }

  edges_ = graph.edges();

  ss_ << "digraph g{" << std::endl;
  ss_ << "bgcolor=\"transparent\"";
  ss_ << "{" << std::endl;
  ss_ << graph.root()->id() << " [style=filled, fillcolor=blue]" << std::endl;
  for( auto weakN : graph.nodes() )
  {
    auto n = weakN.lock();

    if( n )
    {
      if( n->data().agentId == 0 )
      {
        ss_ << n->id() << " [shape=square, style=filled, fillcolor=" << ( n->id() == 0 ? "blue" : "cyan" ) << "]" << std::endl;
      }
      else
      {
        ss_ << n->id() << " [shape=circle]" << std::endl;
      }

      if( n->data().nodeType == NodeData::NodeType::OBSERVATION )
      {
        ss_ << n->id() << " [shape=diamond]" << std::endl;
      }
    }
  }
  for( auto weakN : graph.terminalNodes() )
  {
    auto n = weakN.lock();

    if( n )
    {
      ss_ << n->id() << " [style=filled, fillcolor=green]" << std::endl;
    }
  }
  ss_ << "}" << std::endl;

  saveGraphFrom( graph.root() );

  ss_ << "}" << std::endl;
}

void GraphPrinter::saveGraphFrom( const DecisionGraph::GraphNodeType::ptr & node )
{
  if( printedNodes_.count( node->id() ) != 0 )
  {
    return;
  }
  else
  {
    printedNodes_.insert( node->id() );
  }

  for( auto c : node->children() )
  {
    std::stringstream ss;
    std::string label;

    auto edge = edges_[ c->id()][ node->id() ];
    auto p = edge.first;
    auto leadingArtifact = edge.second;

    if( node->data().nodeType == NodeData::NodeType::ACTION )
    {
      auto agentLabel = agentPrefix_ + std::to_string( node->data().agentId ) + agentSuffix_;
      auto actionLabel = extractActionLabel( leadingArtifact, node->data().agentId );

      boost::replace_all(agentLabel, "__", "");
      boost::replace_all(actionLabel, "(", "");
      boost::replace_all(actionLabel, ")", "");

      ss << agentLabel << std::endl;
      ss << actionLabel;

      label = ss.str();
      boost::replace_all(label, "{", "");
    }
    else
    {
      if( ! leadingArtifact.empty() )
      {
        ss << leadingArtifact << std::endl;
      }
      ss << p;

      label = ss.str();
    }

    ss_ << node->id() << "->" << c->id() << " [ label=\"" << label << "\" ]" << ";" << std::endl;

    saveGraphFrom( c );
  }
}

std::string GraphPrinter::extractActionLabel( const std::string & leadingArtifact, uint agentId ) const
{
  auto agentLabel = agentPrefix_ + std::to_string( agentId ) + agentSuffix_;

  if( leadingArtifact.find( agentLabel ) != -1 )
  {
    return leadingArtifact.substr( agentLabel.size() + 1, leadingArtifact.size() );
  }
  else
  {
    return leadingArtifact;
  }
}

}
