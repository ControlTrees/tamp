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
#include <list>

#include <policy.h>

#include "po_node.h"

namespace tp
{

class PolicyBuilder
{
public:
  PolicyBuilder( PONode::ptr root );

  Policy::ptr getPolicy() const;

private:
  void process( PONode::ptr node );

private:
  Policy::ptr policy_;
  std::map< PONode::ptr, PolicyNode::ptr > PO2Policy_;
};

}
