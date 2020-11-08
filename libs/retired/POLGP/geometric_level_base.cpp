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

#include "polgp_node.h"

GeometricLevelBase::GeometricLevelBase( POLGPNode * node, rai::String const& name, const KOMOFactory & komoFactory )
  : name_( name )
  , N_( node->N() )
  , node_( node )
  , komoFactory_( komoFactory )
  , costs_( N_ )
  , constraints_( N_ )
  , solved_( N_ )
  , infeasibles_( N_ )
  , komos_( N_ )
  , isTerminal_( false )
  , isSolved_( false )
  , isInfeasible_( false )
{
  for( auto w = 0; w < N_; ++w )
  {
    costs_( w ) = 0;
    constraints_( w ) = 0;
    solved_( w ) = false;
    infeasibles_( w ) = false;
  }
}
