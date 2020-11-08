#pragma once

#include <string>
#include <unordered_map>
#include <future>

#include <skeleton.h>

#include <komo_factory.h>
#include <komo_planner_config.h>
#include <komo_wrapper.h>
#include <komo_sparse_planner.h>

#include <HessianDecomposition/hessian_decomposition.h>

namespace mp
{

class KOMOSubProblemsFinder : KOMOSparsePlanner
{
public:
  KOMOSubProblemsFinder(const KOMOPlannerConfig& config, const KOMOFactory& factory)
    : KOMOSparsePlanner(config, factory) {};

  hessian_decomposition::Decomposition analyse(Policy &, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > &);
  void optimize( Policy &, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & ) const override{};
};

// Subproblems set-up by a x mask, will adapt the objectives var, and komo time, to minimize computation time and memory
//struct ADMM_MotionProblem_Compressor : KOMO::Conv_MotionProblem_GraphProblem
//{
//  ADMM_MotionProblem_Compressor(KOMO& _komo) : Conv_MotionProblem_GraphProblem(_komo) {}

//  void setSubProblemXMask(const intA & _tmask) { tmask = _tmask; }
//  void compress(); // use mask and

//  virtual void getStructure(uintA& variableDimensions, intAA& featureVariables, ObjectiveTypeA& featureTypes) override;
//  virtual void phi(arr& phi, arrA& J, arrA& H, const arr& x) override;

//  void getXMask(arr & xmask) const; //   for(uint t=0; t<T; t++) x.append(configurations(t+k_order)->getJointState());

//  intA tmask; // mask per step
//};
}
