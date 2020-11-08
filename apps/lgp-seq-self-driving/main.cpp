#include <KOMO/komo.h>

#include <observation_tasks.h>
//#include <approx_point_to_shape.h>
#include <axis_bound.h>
#include <axis_distance.h>
#include <komo_factory.h>
#include <car_kinematic.h>

using namespace std;

//===========================================================================

void overtake()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( rai::KinematicWorld( "model_overtake.g" ) );


  // general settings
  komo.setSquaredQAccelerations();
  //komo.addObjective( 0.0, -1, new CarKinematic( "car_ego" ), OT_eq, NoArr, 1e2, 1 );

  // road bounds
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.15} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.15} );

  // min speed
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // get sight
  komo.addObjective( 2.0, 3.0, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos );

  // overtake constraints
  komo.setPosition( 4.0, -1, "car_ego", "truck", OT_sos, { 0.6, 0, 0 } );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_op" );

  komo.add_collision( true, 0.03 );

  //rai::wait( 30, true );

  // launch komo
  komo.reset();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  //komo.plotTrajectory();
  //komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void carkin()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( rai::KinematicWorld( "model_carkin.g" ) );

  // general settings
  komo.setSquaredQAccelerations();
  komo.addObjective( 0.0, -1, new CarKinematic( "car_ego", komo.world ), OT_eq, NoArr, 1e2, 1 );

  // road bounds
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.15} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.15} );

  // min speed
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // get sight
  komo.addObjective( 2.0, 3.0, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos );

  // overtake constraints
  komo.setPosition( 5.0, -1, "car_ego", "truck", OT_sos, { 0.6, 0, 0 } );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );

  // opposite car speed
  arr op_speed{ -0.00, 0, 0 };
  komo.setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_op" );
  //komo.activateCollisions( "car_ego", "car_op_2" );


  komo.add_collision( true, 0.03 );

  //rai::wait( 30, true );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  komo.plotTrajectory();
  komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void attempt()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( rai::KinematicWorld( "model_attempt.g" ) );

  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.15} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.15} );

  // min speed
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // get sight
  komo.addObjective( 2.0, 3.0, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos );

  // move back constraints
  komo.setPosition( 4.0, -1, "car_ego", "truck", OT_sos, { -0.6, 0, 0 } );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_op" );
  //komo.activateCollisions( "car_ego", "car_op_2" );


  komo.add_collision( true, 0.03 );

  //rai::wait( 30, true );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  komo.plotTrajectory();
  komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void cooperative()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( rai::KinematicWorld( "model_cooperative.g" ) );

  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos, {-0.075} ); // , 0.1
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "car_op", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_op", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos, {0.075} ); // , 0.1
  komo.addObjective( 0.0, -1, new AxisBound( "car_op", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  // min speed
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // get sight
  komo.addObjective( 2.0, 3.0, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos );

  // overtake constraints
  komo.setPosition( 4.0, -1, "car_ego", "truck", OT_sos, { 0.6, 0, 0 } );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_op" );
  //komo.activateCollisions( "car_ego", "car_op_2" );

  komo.add_collision( true, 0.03 );

  //rai::wait( 30, true );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  komo.plotTrajectory();
  komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void cooperative_2()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( rai::KinematicWorld( "model_cooperative_2.g" ) );

  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos, {-0.075} ); // , 0.1
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "truck_2", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "truck_2", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos, {-0.075} ); // , 0.1
  komo.addObjective( 0.0, -1, new AxisBound( "truck_2", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "car_op", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_op", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos, {0.075} ); // , 0.1
  komo.addObjective( 0.0, -1, new AxisBound( "car_op", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  // min speed
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // get sight
  komo.addObjective( 2.0, 3.0, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos );

  // overtake constraints
  komo.setPosition( 4.0, -1, "car_ego", "truck", OT_sos, { 0.4, 0, 0 } );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );
  komo.addObjective( 0.0, -1, new AxisBound( "truck_2", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // distance between vehicle
  //komo.addObjective( 0.0, -1, new AxisDistance( "truck_2", "car_ego", 0.5, AxisDistance::Y, AxisDistance::MIN ), OT_ineq );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "truck_2" );
  komo.activateCollisions( "car_ego", "car_op" );
  //komo.activateCollisions( "car_ego", "car_op_2" );


  komo.add_collision( true, 0.05 );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  komo.plotTrajectory();
  komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void cooperative_3()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( rai::KinematicWorld( "model_cooperative_3.g" ) );

  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos, {-0.075} ); // , 0.1
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "truck_2", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "truck_2", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos, {-0.075} ); // , 0.1
  komo.addObjective( 0.0, -1, new AxisBound( "truck_2", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "car_op", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_op", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos, {0.075} ); // , 0.1
  komo.addObjective( 0.0, -1, new AxisBound( "car_op", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  // min speed
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // get sight
  komo.addObjective( 3.0, 3.0, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos );

  // overtake constraints
  //komo.setPosition( 5.0, -1, "car_ego", "truck", OT_sos, { 0.4, 0, 0 } );
  komo.setPosition( 5.0, -1, "car_ego", "truck_2", OT_sos, { 0.4, 0, 0 } );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  komo.setVelocity( 0.0, 0.1, "truck", NULL, OT_eq, truck_speed );
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::X, AxisBound::MAX, komo.world ), OT_ineq,  arr{ 0.04 }, 1e2, 1 );
  komo.addObjective( 0.0, -1, new AxisBound( "truck_2", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // distance between vehicle
  //komo.addObjective( 0.0, -1, new AxisDistance( "truck_2", "car_ego", 0.5, AxisDistance::Y, AxisDistance::MIN ), OT_ineq );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "truck_2" );
  komo.activateCollisions( "car_ego", "car_op" );
  //komo.activateCollisions( "car_ego", "car_op_2" );


  komo.add_collision( true, 0.05 );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  komo.plotTrajectory();
  komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void cooperative_3_bis()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( rai::KinematicWorld( "model_cooperative_3.g" ) );

  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos, {-0.075} ); // , 0.1
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "truck_2", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "truck_2", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos, {-0.075} ); // , 0.1
  komo.addObjective( 0.0, -1, new AxisBound( "truck_2", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "car_op", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_op", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos, {0.075} ); // , 0.1
  komo.addObjective( 0.0, -1, new AxisBound( "car_op", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  // min speed
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // get sight
  komo.addObjective( 3.0, 3.0, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_sos );

  // overtake constraints
  //komo.setPosition( 5.0, -1, "car_ego", "truck", OT_sos, { 0.4, 0, 0 } );
  komo.setPosition( 7.0, -1, "car_ego", "truck_2", OT_sos, { 0.4, 0, 0 } );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  komo.setVelocity( 0.0, 0.1, "truck", NULL, OT_eq, truck_speed );
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::X, AxisBound::MAX, komo.world ), OT_ineq,  arr{ 0.04 }, 1e2, 1 );
  komo.addObjective( 0.0, -1, new AxisBound( "truck_2", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // distance between vehicle
  //komo.addObjective( 0.0, -1, new AxisDistance( "truck_2", "car_ego", 0.5, AxisDistance::Y, AxisDistance::MIN ), OT_ineq );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

  komo.activateCollisions( "truck", "truck_2" );
  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "truck_2" );
  komo.activateCollisions( "car_ego", "car_op" );
  //komo.activateCollisions( "car_ego", "car_op_2" );


  komo.add_collision( true, 0.05 );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  komo.plotTrajectory();
  komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void cooperative_4()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( rai::KinematicWorld( "model_cooperative_4.g" ) );

  //komo.world.watch( true );
  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "car_1", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_1", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "car_2", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_2", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "car_3", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_3", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  //komo.addObjective( 0.0, -1, new AxisBound( "truck", -0.075, AxisBound::Y, AxisBound::MIN, 0.1 ), OT_sos );
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  // min speed
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.06 }, 1e2, 1 );

  // get sight
  //komo.addObjective( 3.0, 3.0, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sos );

  // overtake constraints
  //komo.setPosition( 5.0, -1, "car_ego", "truck", OT_sos, { 0.4, 0, 0 } );
  //komo.setPosition( 7.0, -1, "car_ego", "truck_2", OT_sos, { 0.4, 0, 0 } );

  // vehicle speeds
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::X, AxisBound::MAX, komo.world ), OT_ineq,  arr{ 0.045 }, 1e2, 1 );

  komo.addObjective( 0.0, -1, new AxisBound( "car_1", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );
  komo.addObjective( 0.0, -1, new AxisBound( "car_1", AxisBound::X, AxisBound::MAX, komo.world ), OT_ineq,  arr{ 0.045 }, 1e2, 1 );

  komo.addObjective( 0.0, -1, new AxisBound( "car_2", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );
  komo.addObjective( 0.0, -1, new AxisBound( "car_2", AxisBound::X, AxisBound::MAX, komo.world ), OT_ineq,  arr{ 0.045 }, 1e2, 1 );

  komo.addObjective( 0.0, -1, new AxisBound( "car_3", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );
  komo.addObjective( 0.0, -1, new AxisBound( "car_3", AxisBound::X, AxisBound::MAX, komo.world ), OT_ineq,  arr{ 0.04 }, 1e2, 1 );


  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_1" );
  komo.activateCollisions( "car_ego", "car_2" );
  komo.activateCollisions( "car_ego", "car_3" );

  komo.activateCollisions( "truck", "car_1" );
  komo.activateCollisions( "truck", "car_2" );
  komo.activateCollisions( "truck", "car_3" );

  komo.activateCollisions( "car_1", "car_2" );
  komo.activateCollisions( "car_1", "car_3" );

  komo.activateCollisions( "car_2", "car_3" );

  komo.add_collision( true, 0.05 );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  komo.plotTrajectory();
  komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void cooperative_5()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( rai::KinematicWorld( "model_cooperative_5.g" ) );

  //komo.world.watch( true );
  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "car_1", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_1", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "car_2", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_2", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "car_3", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_3", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  //komo.addObjective( 0.0, -1, new AxisBound( "truck", -0.075, AxisBound::Y, AxisBound::MIN, 0.1 ), OT_sos );
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  // min speed
  //komo.addObjective( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.06 }, 1e2, 1 );
  komo.setVelocity( 0.0, -1, "car_ego", NULL, OT_eq, { 0.07, 0, 0 } );

  // get sight
  //komo.addObjective( 3.0, 3.0, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sos );

  // overtake constraints
  //komo.setPosition( 5.0, -1, "car_ego", "truck", OT_sos, { 0.4, 0, 0 } );
  //komo.setPosition( 7.0, -1, "car_ego", "truck_2", OT_sos, { 0.4, 0, 0 } );

  // vehicle speeds
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, { 0.03, 0, 0 } );

  //komo.setVelocity( 0.0, -1, "car_1", NULL, OT_sos, { 0.03, 0, 0 } );
  //komo.setVelocity( 0.0, -1, "car_2", NULL, OT_sos, { 0.03, 0, 0 } );
  //komo.setVelocity( 0.0, -1, "car_3", NULL, OT_sos, { 0.03, 0, 0 } );


  komo.addObjective( 0.0, -1, new AxisBound( "car_1", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  komo.addObjective( 0.0, -1, new AxisBound( "car_1", AxisBound::X, AxisBound::MAX, komo.world ), OT_ineq,  arr{ 0.045 }, 1e2, 1 );

  komo.addObjective( 0.0, -1, new AxisBound( "car_2", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );
  komo.addObjective( 0.0, -1, new AxisBound( "car_2", AxisBound::X, AxisBound::MAX, komo.world ), OT_ineq,  arr{ 0.045 }, 1e2, 1 );

  komo.addObjective( 0.0, -1, new AxisBound( "car_3", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );
  komo.addObjective( 0.0, -1, new AxisBound( "car_3", AxisBound::X, AxisBound::MAX, komo.world ), OT_ineq,  arr{ 0.045 }, 1e2, 1 );


  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_1" );
  komo.activateCollisions( "car_ego", "car_2" );
  komo.activateCollisions( "car_ego", "car_3" );

  komo.activateCollisions( "truck", "car_1" );
  komo.activateCollisions( "truck", "car_2" );
  komo.activateCollisions( "truck", "car_3" );

  komo.activateCollisions( "car_1", "car_2" );
  komo.activateCollisions( "car_1", "car_3" );

  komo.activateCollisions( "car_2", "car_3" );

  komo.add_collision( true, 0.05 );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  komo.plotTrajectory();
  komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void cooperative_flying()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( rai::KinematicWorld( "model_cooperative_flying.g" ) );

  //komo.world.watch( true );
  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "car_1", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_1", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "car_2", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_2", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "car_3", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_3", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.12} );
  //komo.addObjective( 0.0, -1, new AxisBound( "truck", -0.075, AxisBound::Y, AxisBound::MIN, 0.1 ), OT_sos );
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.12} );

  // min speed
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.05 }, 1e2, 1 );

  // get sight
  //komo.addObjective( 3.0, 3.0, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sos );

  // overtake constraints
  //komo.setPosition( 5.0, -1, "car_ego", "truck", OT_sos, { 0.4, 0, 0 } );
  //komo.setPosition( 7.0, -1, "car_ego", "truck_2", OT_sos, { 0.4, 0, 0 } );

  // vehicle speeds
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  komo.addObjective( 0.0, -1, new AxisBound( "truck", AxisBound::X, AxisBound::MAX, komo.world ), OT_ineq,  arr{ 0.04 }, 1e2, 1 );

  komo.addObjective( 0.0, -1, new AxisBound( "car_1", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  //komo.addObjective( 0.0, -1, new AxisBound( "car_1",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.04 }, 1e2, 1 );

  komo.addObjective( 0.0, -1, new AxisBound( "car_2", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  //komo.addObjective( 0.0, -1, new AxisBound( "car_2",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.04 }, 1e2, 1 );

  komo.addObjective( 0.0, -1, new AxisBound( "car_3", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  //komo.addObjective( 0.0, -1, new AxisBound( "car_3",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.04 }, 1e2, 1 );


  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_1" );
  komo.activateCollisions( "car_ego", "car_2" );
  komo.activateCollisions( "car_ego", "car_3" );

  komo.activateCollisions( "truck", "car_1" );
  komo.activateCollisions( "truck", "car_2" );
  komo.activateCollisions( "truck", "car_3" );

  komo.activateCollisions( "car_1", "car_2" );
  komo.activateCollisions( "car_1", "car_3" );

  komo.activateCollisions( "car_2", "car_3" );

  komo.add_collision( true, 0.05 );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  komo.plotTrajectory();
  komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void lane_insertion()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( rai::KinematicWorld( "model_lane_insertion.g" ) );

  //komo.world.watch( true );
  // general settings
  komo.setSquaredQAccelerations();

  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MIN, komo.world ), OT_ineq, {-0.15} );
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::Y, AxisBound::MAX, komo.world ), OT_ineq, {0.15} );

  // min speed
  komo.addObjective( 0.0, -1, new AxisBound( "car_ego", AxisBound::X, AxisBound::MIN, komo.world ), OT_ineq, - arr{ 0.025}, 1e2, 1 );

  // overtake constraints
  komo.setPosition( 7.0, -1, "car_ego", "car_1", OT_sos, { 0.225, 0, 0 } );
  //komo.setPosition( 10, -1, "car_ego", "truck", OT_sos, { 0.475, 0, 0 } );

  // other vehicles speeds
  arr k_speed{ 0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, k_speed );
  komo.setVelocity( 0.0, -1, "car_1", NULL, OT_eq, k_speed );
  komo.setVelocity( 0.0, -1, "car_2", NULL, OT_eq, k_speed );
  komo.setVelocity( 0.0, -1, "car_3", NULL, OT_eq, k_speed );

  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_1" );
  komo.activateCollisions( "car_ego", "car_2" );
  komo.activateCollisions( "car_ego", "car_3" );

  komo.add_collision( true, 0.03 );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  komo.plotTrajectory();
  komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void test_config()
{
  //rai::KinematicWorld G( "model_cooperative_5.g" );
  rai::KinematicWorld G( "model_attempt_paper.g" );
  //std::cout << G.q.d0 << std::endl;
  G.watch( true );
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  //carkin();

  //overtake();

  //attempt();

  //cooperative();

  //cooperative_2();

  //cooperative_3();

  //cooperative_3_bis(); // strange

  //cooperative_4();

  //cooperative_5();


  //cooperative_flying();

  //lane_insertion();

  test_config();

  return 0;
}
