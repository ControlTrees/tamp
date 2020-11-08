/*  ------------------------------------------------------------------
    Copyright 2016 Camille Phiquepal
    email: camille.phiquepal@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#pragma once

#include "po_graph_node.h"

namespace tp
{
class POGraph
{
public:
  using ptr = std::shared_ptr< POGraph >;

  POGraph( const POGraphNode::ptr & root, const std::list < POGraphNode::ptr > & terminals )
    : root_( root )
    , terminals_( terminals )
    , indexedNodes_( root_->shared_node_list()->size() )
  {
    for( auto n : *root_->shared_node_list() )
    {
      indexedNodes_[ n->id() ] = n;
    }
  }

  POGraphNode::ptr root() const
  {
    return root_;
  }

  std::list < POGraphNode::ptr > terminals() const
  {
    return terminals_;
  }

  POGraphNode::ptr getNode( const std::size_t i ) const
  {
    return indexedNodes_[ i ];
  }

  std::size_t size() const { return indexedNodes_.size(); }

private:
  POGraphNode::ptr root_;
  std::list  < POGraphNode::ptr > terminals_;
  std::vector< POGraphNode::ptr > indexedNodes_;
};

class POWeightedGraph
{
public:
  using ptr = std::shared_ptr< POWeightedGraph >;

public:
  POWeightedGraph( const POGraph::ptr & graph, double r )
    : graph_( graph )
    , size_( graph->size() )
    , initialReward_( r )
    , rewards_( size_ * size_, m_inf() )
    , n_update_( size_ * size_, 0 )
    , accessible_( size_, 0 )
  {
    reset();
  }

  void reset()
  {
    rewards_    = std::vector< double >( size_ * size_, m_inf() );
    accessible_ = std::vector< uint >( size_, 0 );

    std::list< POGraphNode::ptr > parents;
    parents.push_back( graph_->root() );

    while( ! parents.empty() )
    {
      auto parent = parents.back();
      parents.pop_back();

      accessible_[ parent->id() ] = 1;

      for( auto f : parent->families() )
      {
        for( auto c : f )
        {
          auto i = index( parent->id(), c->id() );

          rewards_[ i ] = initialReward_;

          if( ! accessible_[ c->id() ] )  // push if it has not been flagged accessible yet
          {
            parents.push_front( c );
          }
        }
      }
    }
  }

  void removeEdge( const std::size_t parent, const std::size_t child )
  {
    auto i = index( parent, child );

    rewards_[ i ] = m_inf();

    auto n = graph_->getNode( child );

    // set child as not accessible if there are no edges to the child
    bool isAccessible = false;
    for( auto p : n->parents() )
    {
      isAccessible = isAccessible || edgePossible( p->id(), n->id() );
    }

    if( ! isAccessible ) accessible_[ child ] = -1;
  }

  void setReward( const std::size_t parent, const std::size_t child, double r )
  {
    CHECK( r != -0.4, "" );

    //CHECK( r > m_inf(), "use remove Edge to delete an edge" );
    if( r == m_inf() )  // early stop if infeasibility
    {
      removeEdge( parent, child );
      return;
    }

    auto i = index( parent, child );
    auto k = n_update_[ i ];

//    CHECK( rewards_[ i ] != r, "" );

//    if( k > 1 && fabs( r ) / fabs( rewards_[ i ] ) > 10.0 )  // outlier rejection
//    {
//      return;
//    }

    n_update_[ i ]++;

    rewards_[ i ] = ( k * rewards_[ i ] + r ) / ( k + 1 );
  }

  void removeNode( const std::size_t nodeId )
  {
    auto n = graph_->getNode( nodeId );

    for( auto p : n->parents() )
    {
      removeEdge( p->id(), n->id() );
    }

    accessible_[ nodeId ] = -1;
  }

  bool edgePossible( const std::size_t parent, const std::size_t child ) const
  {
    auto i = index( parent, child );

    return rewards_[ i ] > m_inf();
  }

  bool nodeAccessible( const std::size_t id ) const
  {
    return accessible_[ id ];
  }

  double reward( const std::size_t parent, const std::size_t child ) const
  {
    auto i = index( parent, child );

    return rewards_[ i ];
  }

  //
  POGraphNode::ptr getNode( const std::size_t i ) const
  {
    return graph_->getNode( i );
  }

  POGraphNode::ptr root() const
  {
    return graph_->root();
  }

  std::list < POGraphNode::ptr > terminals() const
  {
    return graph_->terminals();
  }

  std::size_t size() const { return graph_->size(); }

  POWeightedGraph::ptr clone() const
  {
    auto copy = std::make_shared< POWeightedGraph >( graph_, initialReward_ );
    copy->rewards_    = rewards_;
    copy->accessible_ = accessible_;

    return copy;
  }

private:
  std::size_t index( const std::size_t parent, const std::size_t child ) const
  {
    return size_ * parent + child;
  }

private:
  POGraph::ptr graph_;
  const std::size_t size_;
  const double initialReward_;
  std::vector< double > rewards_;
  std::vector< uint >   n_update_;
  std::vector< uint >   accessible_;
};
}
