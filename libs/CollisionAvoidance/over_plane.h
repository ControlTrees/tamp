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

#include <math_utility.h>

#include <Kin/feature.h>
#include <Kin/taskMaps.h>

using namespace std;

struct OverPlaneConstraint:Feature
{
  OverPlaneConstraint( const rai::KinematicWorld& G, const char* iBobyName, const char* jPlaneBodyName, double _margin=.02 )
    : iBobyName_     ( iBobyName )
    , jPlaneBodyName_( jPlaneBodyName )
    , margin_( _margin )
  {
    //collisionModel_.append( rai::Vector( 0, 0, 0 ) );

    // tmp camille is only temporary, get voxels from the shape
    collisionModel_.append( rai::Vector( -0.15, -0.15, 0 ) );
    collisionModel_.append( rai::Vector( -0.15,  0.15, 0 ) );
    collisionModel_.append( rai::Vector(  0.15, -0.15, 0 ) );
    collisionModel_.append( rai::Vector(  0.15, 0.15, 0 ) );
  }

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G, int t)
  {
    auto body = G.getFrameByName( iBobyName_ );
    auto plane = G.getFrameByName( jPlaneBodyName_ );

    arr positionPLane;
    arr positionJPLane;
    G.kinematicsPos( positionPLane, positionJPLane, plane );

    arr tmp_y = zeros( collisionModel_.N );
    arr tmp_J = zeros( collisionModel_.N, positionJPLane.d1 );

    for( auto w = 0; w < collisionModel_.N; ++w )
    {
      arr _y;
      arr _Jy;
      G.kinematicsPos( _y, _Jy, body, collisionModel_( w ) );
//      std::cout << w << std::endl;
//      std::cout << _Jy << std::endl;

      double md = positionPLane( 2 ) -_y( 2 ) + margin_;

      tmp_y( w ) = md;

//      std::cout << "container_1:" << _y << std::endl;

//      std::cout << "table:" << positionTable << std::endl;
      //std::cout << "md:" << md << std::endl;

      for( auto i = 0; i < tmp_J.d1; ++i )
      {
          tmp_J( w, i ) = positionJPLane( 2, i ) - _Jy( 2, i );
      }
    }

    // commit results
    y = tmp_y;
    if(&J) J = tmp_J;
  }

  rai::String shortTag(const rai::KinematicWorld& G){ return STRING("OverTableConstraint"); }

  uint dim_phi(const rai::KinematicWorld& G)
  {
    return collisionModel_.N;// + constantVectors_.N;
  }

private:
  rai::String iBobyName_;
  rai::String jPlaneBodyName_;
  double margin_;
  rai::Array< rai::Vector > collisionModel_;
  //rai::Array< rai::Vector > constantVectors_;

};
