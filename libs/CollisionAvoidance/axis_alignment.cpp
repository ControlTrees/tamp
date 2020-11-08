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

#include <axis_alignment.h>

//-----AxisAlignment----------------//

void AxisAlignment::phi( arr& y, arr& J, const rai::KinematicWorld& G, int t )
{
  arr tmp_y = zeros( 1 );
  arr tmp_J = zeros( 1, G.q.N );

  auto body = G.getFrameByName( bobyName_ );

  arr bodyAxisDirection, bodyJAxisDirection;
  G.kinematicsVec( bodyAxisDirection, bodyJAxisDirection, body, axis_ );

  double dot_product = dot( bodyAxisDirection, axis_ );

  double cost = 1 - dot_product;

  tmp_y( 0 ) = cost;
  tmp_J.setMatrixBlock( - ( ~ axis_ ) * bodyJAxisDirection, 0, 0 );

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}
