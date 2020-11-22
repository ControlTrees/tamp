#include <komo_sub_problems_finder.h>
#include <tree_builder.h>
#include <komo_wrapper.h>
#include <trajectory_tree_visualizer.h>
#include <decentralized_optimizer.h>

#include <Kin/kin.h>
#include <Kin/switch.h>
#include <Kin/TM_transition.h>
#include <Kin/TM_FlagConstraints.h>
#include <Kin/TM_FixSwitchedObjects.h>

//#include <dlib/clustering.h>

#include <HessianDecomposition/hessian_decomposition.h>

namespace mp
{
/// SUBPROBLEMS ANALYSER COMPRESSED
hessian_decomposition::Decomposition KOMOSubProblemsFinder::analyse(Policy & policy, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics)
{
  using W = KomoWrapper;

  // build tree
  auto tree = buildTree(policy);

  // prepare komo
  auto komo = intializeKOMO(tree, startKinematics.front());

  // ground policy actions
  komo->groundInit(tree);
  auto allVars = getSubProblems(tree, policy);
  groundPolicyActionsJoint(tree, policy, komo);

  // run optimization
  komo->verbose = 3;
  W(komo.get()).reset(allVars);

  auto start = std::chrono::high_resolution_clock::now();

  komo->run();
  const auto & H = komo->opt->newton.Hx;

  auto decomp = hessian_decomposition::decomposeSparseHessian(H, H.d0 / 2 + 1, 3);

  // create as mayn komos as subpbs
  // based on decomp, adjust the vars of each objectives
  // re-extract xmasks
  // optimize

  return decomp;
}

///MOTION PROBLEM
//void ADMM_MotionProblem_Compressor::getStructure(uintA& variableDimensions, intAA& featureVariables, ObjectiveTypeA& featureTypes)
//{
//  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet: use komo.reset()");
//  if(!!variableDimensions) {
//    variableDimensions.resize(komo.T);
//    for(uint t=0; t<komo.T; t++)
//      variableDimensions(t) = komo.configurations(t+komo.k_order)->getJointStateDimension();
//  }

//  if(!!featureVariables) featureVariables.clear();
//  if(!!featureTypes) featureTypes.clear();
//  uint M=0;
//  for(Objective *ob:komo.objectives) {
//    CHECK_EQ(ob->vars.nd, 2, "in sparse mode, vars need to be tuples of variables");
//    for(uint t=0;t<ob->vars.d0;t++)
//      if(tmask(ob->vars(t, -1)))
//      {
//        WorldL Ktuple = komo.configurations.sub(convert<uint,int>(ob->vars[t]+(int)komo.k_order));
//        uint m = ob->map->__dim_phi(Ktuple); //dimensionality of this task
//        if(!!featureVariables) featureVariables.append(ob->vars[t], m);
//        if(!!featureTypes) featureTypes.append(ob->type, m);
//        M += m;
//      }
//  }

//  if(!!featureTypes) komo.featureTypes = featureTypes;

//  dimPhi = M;
//}

//void ADMM_MotionProblem_Compressor::phi(arr& phi, arrA& J, arrA& H, const arr& x)
//{
//  //-- set the trajectory
//  komo.set_x(x);

////  if(!dimPhi) getStructure();
//  CHECK(dimPhi,"getStructure must be called first");
//  phi.resize(dimPhi);
//  if(!!J) J.resize(dimPhi);

////  uintA x_index = getKtupleDim(komo.configurations({komo.k_order,-1}));
////  x_index.prepend(0);

//  rai::timerStart();
//  arr y, Jy;
//  uint M=0;
//  for(Objective *ob:komo.objectives) {
//    CHECK_EQ(ob->vars.nd, 2, "in sparse mode, vars need to be tuples of variables");
//    for(uint t=0;t<ob->vars.d0;t++)
//    if(tmask(ob->vars(t, -1)))
//    {
//      const auto scale = ob->scales.d0 ? ob->scales(t) : 1.0;
//      WorldL Ktuple = komo.configurations.sub(convert<uint,int>(ob->vars[t]+(int)komo.k_order));
//      uintA kdim = getKtupleDim(Ktuple);
//      kdim.prepend(0);

//      //query the task map and check dimensionalities of returns
//      ob->map->__phi(y, (!!J?Jy:NoArr), Ktuple);

//      if(!!J) CHECK_EQ(y.N, Jy.d0, "");
//      if(!!J) CHECK_EQ(Jy.nd, 2, "");
//      if(!!J) CHECK_EQ(Jy.d1, kdim.last(), "");
//      if(!y.N) continue;
//      if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

//      //write into phi and J
//      phi.setVectorBlock(scale * y, M); // appy scale on y

//      if(!!J) {
//        for(uint j=ob->vars.d1;j--;){
//          if(ob->vars(t,j)<0){
//            Jy.delColumns(kdim(j),kdim(j+1)-kdim(j)); //delete the columns that correspond to the prefix!!
//          }
//        }
//        for(uint i=0; i<y.N; i++) J(M+i) = scale * Jy[i]; // appy scale on J
//      }

//      //counter for features phi
//      M += y.N;
//    }
//  }

//  komo.timeFeatures += rai::timerRead(true);

//  CHECK_EQ(M, dimPhi, "");
//  //  if(!!lambda) CHECK_EQ(prevLambda, lambda, ""); //this ASSERT only holds is none of the tasks is variable dim!
//  komo.featureValues = phi;
//  komo.featureDense = true;
//}

//void ADMM_MotionProblem_Compressor::getXMask(arr & xmask) const
//{
//  CHECK(komo.x.d0 > 0, "should reset first!");

//  xmask = zeros(komo.x.d0);

//  uint s=0;
//  std::vector<std::pair<uint, uint>> t_to_x_interval(komo.T);
//  for(uint t=0; t<komo.T; t++)// x.append(configurations(t+k_order)->getJointState());
//  {
//    uint n = komo.configurations(t+komo.k_order)->getJointStateDimension();
//    t_to_x_interval[t] = std::pair<uint, uint>(s, s+n);
//    s+=n;
//  }

//  for(Objective *ob:komo.objectives)
//  {
//    for(uint t=0;t<ob->vars.d0;t++)
//    {
//      auto global = ob->vars(t, -1);
//      if(tmask(global))
//      {
//        const auto& xinterval = t_to_x_interval[global];
//        for(auto i = xinterval.first; i < xinterval.second; ++i)
//        {
//          xmask(i) = 1.0;
//        }
//      }
//    }
//  }
//}

}
