#include <graph_printer.h>

namespace tp {

void GraphPrinter::print( const POGraphNode::ptr & node, std::list< POGraphNode::ptr > terminals )
{
  if( ! node )
  {
    return;
  }

  ss_ << "digraph g{" << std::endl;
  ss_ << "bgcolor=\"transparent\"";
  ss_ << "{" << std::endl;
  ss_ << node->id() << " [style=filled, fillcolor=blue]" << std::endl;
  for( auto t : terminals )
  {
    ss_ << t->id() << " [style=filled, fillcolor=green]" << std::endl;
  }
  ss_ << "}" << std::endl;

  saveGraphFrom( node );

  ss_ << "}" << std::endl;
}

void GraphPrinter::saveGraphFrom( const POGraphNode::ptr & node )
{
  for( auto f : node->families() )
  {
    if( f.size() == 1 )
    {
      auto c = f.first();

      std::pair< POGraphNode::ptr, POGraphNode::ptr > edge( node, c );

      if( std::find( savedEdges_.begin(), savedEdges_.end(), edge ) == savedEdges_.end() )
      {
        saveActionEdge( node, c );

        saveGraphFrom( c );
      }
    }
    else
    {
      auto c = f.first();

      std::pair< POGraphNode::ptr, POGraphNode::ptr > edge( node, c );

      if( std::find( savedEdges_.begin(), savedEdges_.end(), edge ) == savedEdges_.end() )
      {
        auto oid = saveObservationNode( node, c );

        for( auto c : f )
        {
          saveObservationEdge( oid, node, c );

          saveGraphFrom( c );
        }
      }
    }
  }
}

std::string GraphPrinter::saveObservationNode( const POGraphNode::ptr & a, const POGraphNode::ptr & b )
{
  std::stringstream ss1;
  ss1 << b->getLeadingActionFromStr( a );

  auto label1 = ss1.str();

  std::string oid = std::to_string( b->id() ) + "'";
  ss_ << "\"" << oid << "\"" << " [ " << "shape=box" << " ] " << ";" << std::endl;
  ss_ << a->id() << "->" << "\"" << oid << "\"" << " [ label=\"" << label1 << "\" ]" << ";" << std::endl;

  return oid;
}

void GraphPrinter::saveActionEdge( const POGraphNode::ptr & a, const POGraphNode::ptr & b )
{
  std::stringstream ss;

  ss << b->getLeadingActionFromStr( a ) << std::endl;;

  auto label = ss.str();

  ss_ << a->id() << "->" << b->id() << " [ label=\"" << label << "\" ]" << ";" << std::endl;

  savedEdges_.push_back( std::pair< POGraphNode::ptr, POGraphNode::ptr >( a, b ) );
}

void GraphPrinter::saveObservationEdge( const std::string & oid, const POGraphNode::ptr & a, const POGraphNode::ptr & b )
{
  std::stringstream ss;

  auto facts =  b->differentiatingFacts();
  for( auto f : facts )
  {
    ss << f << std::endl;
  }

  auto label = ss.str();

  ss_ << "\"" << oid << "\"" << "->" << b->id() << " [ label=\"" << label << "\" ]" << ";" << std::endl;

  savedEdges_.push_back( std::pair< POGraphNode::ptr, POGraphNode::ptr >( a, b ) );
}

}
