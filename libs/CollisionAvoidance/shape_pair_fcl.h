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

/*#pragma once

#include <math_utility.h>

#include <map>

#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/feature.h>
#include <Kin/taskMaps.h>

#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/collision_node.h>
#include <fcl/collision.h>
#include <fcl/distance.h>

using namespace std;

struct ShapePairFCL:Feature
{
  ShapePairFCL(const rai::KinematicWorld& G, const char* iShapeName, const char* jShapeName )
  {
    CHECK( false, "do not use, fcl gives results that are not very usefull, to instable when colliding" );

    i_ = G.getFrameByName( iShapeName )->index;//G.getShapeByName( iShapeName )->index;
    j_ = G.getFrameByName( jShapeName )->index;//G.getShapeByName( jShapeName )->index;
  }

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G, int t);

  void phiProxy( arr& y, arr& J, const rai::KinematicWorld& G, rai::Proxy * p );

  void phiFCL( arr& y, arr& J, const rai::KinematicWorld& G );

  uint dim_phi(const rai::KinematicWorld& G)
  {
    return 1;// + constantVectors_.N;
  }

  rai::String shortTag(const rai::KinematicWorld& G){ return STRING("_ShapePairFCL"); }

private:
  fcl::CollisionObject * createObjectModel( rai::Shape * s );

private:
  uint i_;
  uint j_;

  std::map< rai::Shape *, std::vector<fcl::Vec3f> > vertices_;
  std::map< rai::Shape *, std::vector<fcl::Triangle> > triangles_;
};*/
