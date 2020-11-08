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

#include "po_graph.h"
#include "skeleton.h"

namespace tp
{
class Dijkstra
{
public:
  Dijkstra( const rai::Array< std::shared_ptr<FOL_World> > & folEngines );

  Policy::ptr solve( const POWeightedGraph::ptr & graph, const POGraphNode::ptr & from );

private:
  void dijkstra( const std::list < POGraphNode::ptr > & terminals );
  bool extractSolutionFrom( const POGraphNode::ptr & );
  bool buildPolicy( const POGraphNode::ptr & );
  bool buildPolicyFrom( const POGraphNode::ptr & node, const POGraphNode::ptr & start );
  void checkValuesIntegrity( const Policy::ptr & policy );

private:
  rai::Array< std::shared_ptr< FOL_World > > folEngines_;
  std::vector< double > values_;
  // policy reconstruction
  POWeightedGraph::ptr graph_;
  std::vector< int >   bestFamily_;     // action to take in this bs and i
  std::vector< POGraphNode::ptr > parents_;
  Policy::ptr policy_;
  std::map< POGraphNode::ptr, PolicyNode::ptr > PO2Policy_;
  std::map< PolicyNode::ptr, POGraphNode::ptr > Policy2PO_;
};
}
