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

#include <vertical_velocity.h>


//-----VerticalVelocity----------------//

void VerticalVelocity::phi( arr& y, arr& J, const rai::KinematicWorld& G, int t )
{
  arr tmp_y = zeros( 2 );
  arr tmp_J = zeros( 2, G.q.N );

  auto body = G.getFrameByName( bobyName_ );
  arr p, Jp;
  G.kinematicsPos( p, Jp, body, rai::Vector( 0, 0, 0 ) );

  // commit results
  const double w = 10;
  tmp_y( 0 ) = w * p( 0 );
  tmp_y( 1 ) = w * p( 1 );

  tmp_J.setMatrixBlock( w * Jp.row( 0 ), 0, 0 );
  tmp_J.setMatrixBlock( w * Jp.row( 1 ), 1, 0 );

  //tmp_J.setMatrixBlock( Jp, 0, 0 );

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}
