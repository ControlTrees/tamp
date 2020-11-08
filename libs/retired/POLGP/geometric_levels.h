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

#include "geometric_level_base.h"

struct PoseLevelType : public GeometricLevelBase
{
  PoseLevelType( POLGPNode * node, const KOMOFactory & komoFactory );

  void solve();

  void backtrack();
};

struct PathLevelType : public GeometricLevelBase
{
  PathLevelType( POLGPNode * node, const KOMOFactory & komoFactory, uint microSteps = 10 );

  void solve();

  void backtrack();

private:
  uint microSteps_;
};

struct JointPathLevelType : public GeometricLevelBase
{
  JointPathLevelType( POLGPNode * node, const KOMOFactory & komoFactory, uint microSteps = 10 );

  void solve();

  void backtrack();

private:
  uint microSteps_;
};
