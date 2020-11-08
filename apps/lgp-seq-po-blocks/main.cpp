#include <KOMO/komo-ext.h>

#include <observation_tasks.h>
#include <approx_shape_to_sphere.h>

using namespace std;

//===========================================================================

void move_blocks()
{
  KOMO_ext komo;
  komo.setConfigFromFile();

  // baxter
  rai::String sensorShapeName = "endeffHead";
  arr sensorDir = ARR( 0.0, 0.0, -1.0 );
  rai::String handL = "baxterL";
  rai::String handCL = "baxterCL";
  arr pivotPoint = ARR( 0.05, 0.0, 0.0 );


  // man
//  rai::String sensorShapeName = "manhead";
//  rai::String handL = "handL";
//  rai::String handCL = "humanL";

//  arr sensorDir = ARR( 0.0, -1.0, 0.0 );
//  arr pivotPoint = ARR( 0.0, -0.05, 0.0 );

  {
    arr y;
    arr J;
    auto gs = new ActiveGetSight( sensorShapeName, "block_c", pivotPoint, sensorDir, 0.5 );
    gs->phi( y, J, komo.world );
  }

  //komo.world.watch( true );

  //komo.setSquaredFixJointVelocities();
  komo.setFixEffectiveJoints();
  komo.setFixSwitchedObjects();
  komo.setSquaredQAccelerations();

  ///ALL TIME TASK MAPS
  /*
  komo.setTask( 0.0, 10.0, new TM_AboveBox(komo.world, "block_o", "tableC" ), OT_ineq, NoArr, 1e2);
  komo.setTask( 0.0, 10.0, new TM_AboveBox(komo.world, "block_r", "tableC" ), OT_ineq, NoArr, 1e2);
  komo.setTask( 0.0, 10.0, new TM_AboveBox(komo.world, "block_g", "tableC" ), OT_ineq, NoArr, 1e2);
  komo.setTask( 0.0, 10.0, new TM_AboveBox(komo.world, "block_b", "tableC" ), OT_ineq, NoArr, 1e2);
  */

  komo.addObjective(0.0, 10.0, new ApproxShapeToSphere(komo.world, handCL, "tableC" ), OT_ineq, NoArr, 1e2);

  ///GRASP C, PUT IT on TABLE
  //grasp C
  komo.addSwitch( 2.0, true, "delete",   NULL, "block_c" );
  komo.addSwitch( 2.0, true, "ballZero", handL, "block_c" );

  //observe C
  komo.addObjective( 2.5, 3.0, new ActiveGetSight( sensorShapeName, "block_c", pivotPoint, sensorDir, 0.5 ) );

  //place on table
  komo.setPlace( 4.0, handL, "block_c", "tableC" );

//  ///GRASP O, PUT IT on A
//  //grasp
//  komo.addSwitch( 5.0, true, "delete",   NULL, "block_o" );
//  komo.addSwitch( 5.0, true, "ballZero", "handL", "block_o" );

//  //put on A
//  komo.setPlace( 6.0, "handL", "block_o", "block_a" );

//  ///GRASP C, PUT IT on O
//  //grasp
//  komo.addSwitch( 7.0, true, "delete",   NULL, "block_c" );
//  komo.addSwitch( 7.0, true, "ballZero", "handL", "block_c" );

//  //put on O
//  komo.setPlace( 8.0, "handL", "block_c", "block_o" );

  ///GRASP B, PUT IT on C
  //grasp B
  komo.addSwitch( 5.0, true, "delete",   NULL, "block_b" );
  komo.addSwitch( 5.0, true, "ballZero", handL, "block_b" );

  //observe B
  komo.addObjective( 5.5, 6.0, new ActiveGetSight( sensorShapeName, "block_b", pivotPoint, sensorDir, 0.5 ) );

  //put on C
  komo.setPlace( 7.0, handL, "block_b", "block_c" );

  ///GRASP A, PUT IT on B
  //grasp
  komo.addSwitch( 8.0, true, "delete",   NULL, "block_a" );
  komo.addSwitch( 8.0, true, "ballZero", handL, "block_a" );

  //put on C
  komo.setPlace( 9.0, handL, "block_a", "block_b" );

  // launch komo
  komo.reset();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.3, true);
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  //move_1();

  move_blocks();

  return 0;
}
