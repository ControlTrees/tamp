/*  ------------------------------------------------------------------
    Copyright 2017 Camille Phiquepal
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

#include <Kin/feature.h>

struct AgentKinEquality:Feature{

  AgentKinEquality( uint id, const arr& q/*, const arr& qdot*/ )
    : id_( id )
    , q_  ( q )
    , dim_( q.N )
  {

  }

  virtual void phi( arr& y, arr& J, const rai::KinematicWorld& G, int t=-1 )
  {
    y = zeros( dim_ );
    y.setVectorBlock( G.q - q_, 0 );

    auto tmp_J = eye( dim_, dim_ );

    if(&J) J = tmp_J;
  }

  virtual uint dim_phi( const rai::KinematicWorld& G )
  {
    return dim_;
  }

  virtual rai::String shortTag( const rai::KinematicWorld& G )
  {
    return rai::String( "AgentKinEquality" );
  }

private:
  // parameters
  const uint id_;
  const arr q_;
  const uint dim_;
};
