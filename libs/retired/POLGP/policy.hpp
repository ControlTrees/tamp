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

#include <memory>
#include <Kin/kinViewer.h>
//#include <Motion/motion.h>
#include "polgp_node.h"

namespace tmp
{
class PolicyNode
{
public:
  typedef std::shared_ptr< PolicyNode > ptr;

public:
  void setParent( const PolicyNode::ptr & parent ) { parent_ = parent; }
  void setTime( double t ) { time_ = t; }
private:
  PolicyNode::ptr parent_;
  double time_;
};

class Policy
{
public:
  typedef std::shared_ptr< Policy > ptr;

public:
  virtual ~Policy();

  // modifier
  void init( uint N );
  void setRoot( const PolicyNode::ptr & root ) { root_ = root; }
  void setTrajectory( const WorldL & traj, uint w ) { trajectories_[ w ] = traj; }
  void setCost( double cost ) { cost_ = cost; }

  // getter
  uint N() const { return N_; }
  WorldL getTrajectory( uint w ) const { return trajectories_[ w ]; }
  double cost() const { return cost_; }

private:
  uint N_;
  PolicyNode::ptr root_;
  std::vector< WorldL > trajectories_; // kinematic world over time for each world
  double cost_;
};

// sort nodes so that the ones with the biggest rewards are first
struct PolicyCompare : public std::binary_function<Policy::ptr, Policy::ptr, bool>
{
  bool operator()( Policy::ptr lhs, Policy::ptr rhs) const
  {
    return ! ( lhs->cost() == rhs->cost() ) && ( lhs->cost() < rhs->cost() );
  }
};

class PolicyVisualizer
{
public:
  PolicyVisualizer( const Policy::ptr & policy, const std::string & name );

private:
  std::vector< std::shared_ptr< OrsPathViewer > > views_;
};

class PolicyBuilder
{
public:
  PolicyBuilder( POLGPNode * root );
  Policy::ptr getPolicy() const;

private:
  void process( POLGPNode * node );

private:
  Policy::ptr policy_;

  std::map< POLGPNode *, PolicyNode::ptr > POLGP2Policy_;
};
}

