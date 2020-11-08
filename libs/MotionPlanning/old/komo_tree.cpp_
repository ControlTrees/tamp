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

#include "komo_tree.h"

#include <set>
#include <iomanip>
#include <Gui/opengl.h>
#include <Kin/frame.h>
#include <Core/graph.h>

namespace mp
{

_Branch _Branch::computeMicroStepBranch(const _Branch& a, int stepsPerPhase)
{
  _Branch b;

  b.p = a.p;
  b.leaf_id = a.leaf_id;

  // local
  int n_phases = a.local_to_global.size() - 1;

  b.local_to_global = std::vector< int >(n_phases * stepsPerPhase); // 2 == prefix

  for(int phaseEndIndex = 1; phaseEndIndex <= n_phases; ++phaseEndIndex)
  {
    const uint phaseEnd = a.local_to_global[phaseEndIndex];

    for(int s = 0; s < stepsPerPhase; ++s)
    {
      b.local_to_global[phaseEndIndex * stepsPerPhase - s - 1] = phaseEnd * stepsPerPhase - s -1;
    }
  }

  // global
  int n_global_phases = a.global_to_local.size() - 1;
  b.global_to_local = std::vector< int >(n_global_phases * stepsPerPhase, -1);

  for(int local = 0; local < n_phases * stepsPerPhase; ++local)
  {
    const auto global =b.local_to_global[local];
    b.global_to_local[global] = local;
  }

  // hack
  if( b.local_to_global.size() < 3 )
  {
    b.local_to_global.push_back(2);
    b.global_to_local.push_back(2);
  }

  return b;
}

_Branch _Branch::linearTrajectory(int T)
{
  _Branch b;
  b.p = 1.0;

  b.global_to_local = std::vector< int >(T);
  b.local_to_global = std::vector< int >(T);

  for(auto t = 0; t < T; ++t)
  {
    b.global_to_local[t] = t;
    b.local_to_global[t] = t;
  }

  return b;
}

bool operator==(const _Branch& a, const _Branch& b)
{
  return (a.p == b.p) && (a.local_to_global == b.local_to_global) && (a.global_to_local == b.global_to_local) && (a.leaf_id == b.leaf_id);
}

bool operator<(const _Branch& a, const _Branch& b)
{
  return a.leaf_id < b.leaf_id;
}

//===========================================================================

//#define STEP(p) (floor(branch.pathFromRoot[p]*double(stepsPerPhase) + .500001))-1

double phaseToStep(double p, double stepsPerPhase)
{
  return floor(p*double(stepsPerPhase));//(floor(p*double(stepsPerPhase) + .500001))-1;
}

void TreeTask::setCostSpecs(int fromTime,
                            int toTime, int T,
                            const arr& _target,
                            double _prec){
//#if 0 //MARC_TODO!
  //if(&_target) target = _target; else target = {0.};
  if(fromTime<0) fromTime=0;
  CHECK(toTime>=fromTime,"");
  vars.resize(T).setZero();
//#endif
  for(int local = 0; local < int(branch.local_to_global.size())-2; ++local) // clarify magic number
  {
    if(local >= fromTime && local < toTime)
    {
      auto global = branch.local_to_global[local];
      vars(global) = _prec;
    }
  }
}

void TreeTask::setCostSpecs(double fromTime, double toTime, int stepsPerPhase, uint T, const arr& _target, double _prec, const _Branch& branch_time_spec){
  if(stepsPerPhase<0) stepsPerPhase=T;
  uint maxStepOnBranch = *std::max_element(branch_time_spec.local_to_global.begin(), branch_time_spec.local_to_global.end()) * stepsPerPhase;
  if(phaseToStep(toTime, stepsPerPhase)>maxStepOnBranch){
    LOG(-1) <<"beyond the time!: endTime=" <<toTime <<" phases=" <<double(T)/stepsPerPhase;
  }

  CHECK(&branch_time_spec, "case without branch not handled yet!");
  branch = _Branch::computeMicroStepBranch(branch_time_spec, stepsPerPhase);

  int tFrom = (fromTime<0.?0              :phaseToStep(fromTime, stepsPerPhase));
  int tTo =   (toTime  <0.?maxStepOnBranch:phaseToStep(toTime, stepsPerPhase));
  if(tTo<0) tTo=0;
  if(tFrom>tTo && tFrom-tTo<=(int)map->order) tFrom=tTo;

  setCostSpecs(tFrom, tTo, T, _target, _prec);
}

//==============KOMOTree==============================================

KOMOTree::KOMOTree()
  : KOMO_ext()
  , komo_tree_problem(*this)
{

}

void KOMOTree::run(){
  rai::KinematicWorld::setJointStateCount=0;
  rai::timerStart();
  CHECK(T,"");
  if(opt) delete opt;

  auto cp = Conv_KOMO_Tree_ConstrainedProblem(komo_tree_problem);
  opt = new OptConstrained(x, dual, cp);
  opt->run();

  if(verbose>0){
    cout <<"** optimization time=" <<rai::timerRead()
        <<" setJointStateCount=" <<rai::KinematicWorld::setJointStateCount <<endl;
  }
  if(verbose>1) cout <<getReport(false);
}

bool KOMOTree::checkGradients(){
  CHECK(T,"");

  auto cp = Conv_KOMO_Tree_ConstrainedProblem(komo_tree_problem);

  return checkJacobianCP(cp, x, 1e-4);
}

Objective* KOMOTree::setTask(double startTime, double endTime, Feature *map, ObjectiveType type, const arr& target, double prec, uint order){
  return setTreeTask(startTime, endTime, _Branch::linearTrajectory(T/stepsPerPhase+1), map, type, target, prec, order);
}

TreeTask* KOMOTree::addTreeTask(const char* name, Feature *m, const ObjectiveType& termType){
  TreeTask *t = new TreeTask(m, termType);
  t->name=name;
  tree_tasks.append(t);
  return t;
}

TreeTask *KOMOTree::setTreeTask(double startTime, double endTime, const _Branch& branch, Feature *map, ObjectiveType type, const arr& target, double prec, uint order){
  CHECK(k_order>=order,"");
  map->order = order;
  TreeTask *task = addTreeTask(map->shortTag(world), map, type);
  task->setCostSpecs(startTime, endTime, stepsPerPhase, T, target, prec, branch);
  return task;
}

bool KOMOTree::displayTrajectory(double delay, bool watch){
  const char* tag = "KOMO planned trajectory";

  // retrieve branches
  std::set<mp::_Branch> branches;
  for(auto t: tree_tasks)
  {
    branches.insert(t->branch);
  }

  // name viewer
  auto name = [](uint m, uint leaf_id)
  {
    std::stringstream ss;
    ss << "KOMO display - branch " << m << " - leaf " << leaf_id;
    return ss.str();
  };

  // create viewer for each branch consecutively
  uint branch_number = 0;
  for(const auto branch : branches)
  {
    auto gl = std::make_shared< OpenGL >(name(branch_number, branch.leaf_id).c_str());
    gl->camera.setDefault();
    auto T = branch.local_to_global.size();
    for(uint local = k_order; local < T; ++local)
    {
      auto global = branch.local_to_global[local];
      gl->clear();
      gl->add(glStandardScene, 0);
      gl->addDrawer(configurations(global));
      if(delay<0.){
        if(delay<-10.) FILE("z.graph") <<*configurations(global);
        gl->watch(STRING(tag <<" (time " <<std::setw(3) << local <<'/' <<T <<')').p);
      }else{
        gl->update(STRING(tag <<" (time " <<std::setw(3) << local <<'/' <<T <<')').p);
        if(delay) rai::wait(delay);
      }
    }
    ++branch_number;
  }

  return true;
}

//-- komo problem

void KOMOTree::Conv_Tree_KOMO_Problem::getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes){
  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet: use komo.reset()");
  variableDimensions.resize(komo.T);
  for(uint t=0;t<komo.T;t++) variableDimensions(t) = komo.configurations(t+komo.k_order)->getJointStateDimension();

  featureTimes.clear();
  featureTypes.clear();
  for(uint t=0;t<komo.T;t++){
    for(TreeTask *task: komo.tree_tasks) if(task->isActive(t)){
      //      CHECK(task->prec.N<=MP.T,"");
      const auto local_t = task->to_local_t(t);
      WorldL configurations = getConfigurations(task, local_t);
      uint m = task->map->__dim_phi(configurations); //dimensionality of this task
      featureTimes.append(consts<uint>(task->to_global_t(local_t), m));
     featureTypes.append(consts<ObjectiveType>(task->type, m));
    }
  }
  dimPhi = featureTimes.N;
}

WorldL KOMOTree::Conv_Tree_KOMO_Problem::getConfigurations(TreeTask* task, uint local_t) const
{
  WorldL configurations(komo.k_order+1);
  for(uint k = 0; k<=komo.k_order; ++k)
  {
    configurations(k) = komo.configurations(task->to_global_t(local_t+k));
  }
  return configurations;
}

void KOMOTree::Conv_Tree_KOMO_Problem:: phi(arr& phi, arrA& J, arrA& H, uintA& featureTimes, ObjectiveTypeA& tt, const arr& x, arr& lambda){
  //-- set the trajectory
  komo.set_x(x);

  CHECK(dimPhi,"getStructure must be called first");
  phi.resize(dimPhi);
  if(&tt) tt.resize(dimPhi);
  if(&J)
  {
    J.resize(dimPhi);
    for(uint i=0;i<dimPhi;i++) J(i) = arr(x.N);
  }

  arr y, Jy;
  uint M=0;
  for(uint t=0;t<komo.T;t++){
    for(TreeTask *task: komo.tree_tasks) if(task->isActive(t)){
      //TODO: sightly more efficient: pass only the configurations that correspond to the map->order
      const auto local_t = task->to_local_t(t);
      WorldL configurations = getConfigurations(task, local_t);
      task->map->__phi(y, (&J?Jy:NoArr), configurations);
      if(!y.N) continue;
      if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

      //linear transform (target shift)
#if 0 //MARC_TODO!
      if(task->target.N==1) y -= task->target.elem(0);
      else if(task->target.nd==1) y -= task->target;
      else if(task->target.nd==2) y -= task->target[t];
      y *= task->branch.p * sqrt(task->prec(t));
#endif
      y *= task->branch.p;

      //write into phi and J
      const auto & qN = komo.configurations(0)->q.N;
      phi.setVectorBlock(y, M);
      if(&J){
        Jy *= task->branch.p/* * sqrt(task->prec(t))*/; //MARC_TODO
        for(uint i=0;i<y.N;i++)
        {
          for(uint k=0; k<=komo.k_order; ++k)
          {
            for(uint j=0; j<qN; ++j)
            {
              auto col_in_jacobian = qN * task->to_global_t(local_t+k)+j;
              col_in_jacobian -= get_k() * qN; // shift back to compensate for the prefix

              if(col_in_jacobian < x.N) // case col < 0 implicitely handled by overflow
              {
                J(M+i)(col_in_jacobian)=Jy(i,qN * k + j);
              }
            }
          }
        }
      }
      if(&tt) for(uint i=0;i<y.N;i++) tt(M+i) = task->type;

      //counter for features phi
      M += y.N;
    }
  }

  CHECK_EQ(M, dimPhi, "");
  komo.featureValues = phi;
  if(&tt) komo.featureTypes = tt;
}

//-- converters
Conv_KOMO_Tree_ConstrainedProblem::Conv_KOMO_Tree_ConstrainedProblem(KOMO_Problem& P) : KOMO(P){
  KOMO.getStructure(variableDimensions, featureTimes, featureTypes);
  varDimIntegral = integral(variableDimensions);
}

void Conv_KOMO_Tree_ConstrainedProblem::phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda){
  KOMO.phi(phi, (&J?J_KOMO:NoArrA), (&H?H_KOMO:NoArrA), featureTimes, ot, x, lambda);

  //-- construct J
  if(&J){
    //J = rai::Array<double>(phi.N, x.N);
    J = zeros(phi.N, x.N);

    //loop over features
    for(uint i=0; i<phi.N; i++) {
      arr& Ji = J_KOMO(i);
      CHECK(Ji.N<=J.d1,"");
      memmove(&J(i,0), Ji.p, Ji.sizeT*Ji.N);
    }
  }

  if(&H){
    bool hasFterm = false;
    if(&ot) hasFterm = (ot.findValue(OT_f) != -1);
    if(hasFterm){
      CHECK(H_KOMO.N, "this problem has f-terms -- I need a Hessian!");
      NIY
    }else{
      H.clear();
    }
  }
}

}
