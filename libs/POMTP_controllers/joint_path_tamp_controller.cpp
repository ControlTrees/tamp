#include <joint_path_tamp_controller.h>

#include <set>

Policy JointPathTAMPController::plan(  const TAMPlanningConfiguration & config )
{
  auto skeletonComp = []( const Policy & a, const Policy & b ) { return  a.value() > b.value(); };

  /// LOOP
  std::vector< Policy > markovPolicies;
  std::vector< Policy > jointPolicies;
  Policy policy, previousPolicy;
  tp_.solve();
  policy = tp_.getPolicy();

  uint nIt = 0;

  while( policy != previousPolicy && nIt != config.maxIterations )
  {
    nIt++;

    //policy.load("policy-2-informed.po");

    /// MOTION PLANNING
    auto po     = MotionPlanningParameters( policy.id() );
    po.setParam( "type", "markovJointPath" );
    mp_.solveAndInform( po, policy );

    if( policy.feasible() ) markovPolicies.push_back( policy );
    if( config.saveInformedPolicy ) policy.saveAll( ".", "-informed" );

    /// TASK PLANNING
    tp_.integrate( policy );
    tp_.solve();

    previousPolicy = policy;
    policy = tp_.getPolicy();
  }

  /// JOINT PATH
  std::sort( markovPolicies.begin(), markovPolicies.end(), skeletonComp );
  uint nJointReplanMax = 5;
  uint nJointReplans = 0;
  for( auto policy : markovPolicies )
  {
    auto po     = MotionPlanningParameters( policy.id() );
    po.setParam( "type", "jointPath" );
    mp_.solveAndInform( po, policy );
    jointPolicies.push_back( policy );

    nJointReplans++;
    if( nJointReplans >= nJointReplanMax )
      break;
  }

  std::sort( jointPolicies.begin(), jointPolicies.end(), skeletonComp );
  policy = *jointPolicies.begin();

  /// DIPSLAY
  if( config.saveFinalPolicy ) policy.saveAll( ".", "-final" );

  if( config.showFinalPolicy )
  {
    mp_.display( policy, 3000 );
    rai::wait( config.showDurationSecs, true );
  }

  return policy;
}
