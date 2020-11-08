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

#include "policy.hpp"

namespace tmp
{
Policy::~Policy()
{
  for( auto trajectory : trajectories_ )
  {
    for( auto slice : trajectory )
    {
      if( slice )
      {
        delete slice;
        slice = nullptr;
      }
    }
  }
}

void Policy::init( uint N )
{
  N_ = N;
  trajectories_ = std::vector< WorldL >( N );
}

//---PolicyVisualizer--------//
PolicyVisualizer::PolicyVisualizer( const Policy::ptr & policy, const std::string & name )
{
  views_.resize( policy->N() );
  for( auto w = 0; w < policy->N(); ++w )
  {
    std::string windowName = name + std::string( "-world-" ) + std::to_string( w );
    views_[ w ] = std::make_shared< OrsPathViewer >( windowName.c_str(),  0.1, -0 );
    views_[ w ]->setConfigurations( policy->getTrajectory( w ) );
  }


  threadOpenModules( true );
}

//---PolicyBuilder-----------//

PolicyBuilder::PolicyBuilder( POLGPNode * root )
  : policy_( std::make_shared< Policy >() )
{
  policy_->init( root->N() );

  double cost = 0;

  // retrieve trajectories
  for( auto w = 0; w < root->N(); ++w )
  {
    WorldID wid( w );
    auto node = utility::getTerminalNode( root, wid );

    WorldL kinematicConfigurations = node->jointPathGeometricLevel()->komos()( w )->configurations;
    //WorldL kinematicConfigurations = node->jointPathGeometricLevel()->komos()( w )->configurations;
    WorldL clonedKinematicConfigurations( kinematicConfigurations.d0 );

    // clone the kinematic configurations
    for( auto s = 0; s < kinematicConfigurations.d0; ++s )
    {
      rai::KinematicWorld* clone = new rai::KinematicWorld();
      clone->copy( *(kinematicConfigurations(s)) );
      clonedKinematicConfigurations(s) = clone;
    }

    policy_->setTrajectory( clonedKinematicConfigurations, w );

    // extract the cost
    cost += root->bs()( w ) * node->jointPathGeometricLevel()->cost( w );
  }

  policy_->setCost( cost );
  //

  process( root );
}

void PolicyBuilder::process( POLGPNode * node )
{
  PolicyNode::ptr policyNode = std::make_shared< PolicyNode >();

  if( node->isRoot() )
  {
    policy_->setRoot( policyNode );
  }
  else
  {
    // set parent
    policyNode->setParent( POLGP2Policy_[ node->parent() ] );
  }

  policyNode->setTime( node->time() );

  // save correspondance
  POLGP2Policy_[ node ] = policyNode;

  for( auto c : node->bestFamily() )
  {
    process( c );
  }
}

Policy::ptr PolicyBuilder::getPolicy() const
{
  return policy_;
}
}

