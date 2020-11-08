#include "komo_planner_fixture.h"
#include "komo_sub_problems_finder.h"
#include "komo_wrapper.h"
#include <HessianDecomposition/utils.h>

/////////////////////SINGLE AGENT PARTIALLY OBSERVABLE/////////////////////////////

TEST_F(KomoPlannerSingleAgentFixture, DISABLED_PlanSingleAgent2WJointPath)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-single-agent-2w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointSparse" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Policy::INFORMED );
}

TEST_F(KomoPlannerSingleAgentFixture, DISABLED_HessianDecomp)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-single-agent-2w.po" );

  KOMOSubProblemsFinder pf(planner.config(), planner.komoFactory());
  auto decomp = pf.analyse(policy, planner.startKinematics());
}

TEST_F(KomoPlannerSingleAgentFixture, Foo)
{
  using namespace hessian_decomposition;

  Objective o0(nullptr, OT_sos); // order 0
  o0.vars = intA(5, 1, {0, 2, 4, 6, 8}); // axis 0, order 0

  auto H = buildOneLooselyCoupledProblem();
  auto decomp = decomposeHessian(H, H.d0 / 2 + 1, 2);

  //mp::adaptVar(decomp.problems[0].xmasks.front(), o0);
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

