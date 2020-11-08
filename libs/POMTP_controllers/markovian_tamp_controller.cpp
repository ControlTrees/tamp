#include <markovian_tamp_controller.h>

Policy MarkovianTAMPController::plan( const TAMPlanningConfiguration & config )
{
  /// LOOP
  Policy policy, previousPolicy;
  tp_.solve();
  policy = tp_.getPolicy();

  uint nIt = 0;

  while( policy != previousPolicy && nIt != config.maxIterations )
  {
    nIt++;

    /// MOTION PLANNING
    auto po     = MotionPlanningParameters( policy.id() );
    po.setParam( "type", "markovJointPath" );
    mp_.solveAndInform( po, policy );

    if( config.saveInformedPolicy ) policy.saveAll( ".", "-informed" );

    /// TASK PLANNING
    tp_.integrate( policy );
    tp_.solve();

    previousPolicy = policy;
    policy = tp_.getPolicy();
  }

  if( config.saveFinalPolicy ) policy.saveAll( ".", "-final" );

  if( config.saveFinalPolicy )
  {
    mp_.display( policy, 3000 );
    rai::wait( config.showDurationSecs, true );
  }

  return policy;
}
