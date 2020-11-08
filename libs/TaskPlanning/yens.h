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

#include <po_djikstra.h>

namespace tp
{

class Yens
{
public:
  Yens( const rai::Array< std::shared_ptr<FOL_World> > & folEngines );

  std::list< Policy::ptr > solve( const POWeightedGraph::ptr & graph, uint k );
  std::list< Policy::ptr > solve_( const POWeightedGraph::ptr & graph, uint k );


private:
  POWeightedGraph::ptr graph_;

  rai::Array< std::shared_ptr<FOL_World> > folEngines_;

  Dijkstra dijkstra_;
};

}
