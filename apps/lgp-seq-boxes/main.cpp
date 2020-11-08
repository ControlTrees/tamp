#include <KOMO/komo.h>

#include <observation_tasks.h>
#include <approx_point_to_shape.h>
#include <over_plane.h>
#include <axis_alignment.h>

using namespace std;

//===========================================================================

static void setRigid( double time, rai::String const& object1Name, rai::String const& object2Name, KOMO& komo )
{
  rai::Shape * s1 = komo.world.getShapeByName( object1Name );
  rai::Shape * s2 = komo.world.getShapeByName( object2Name );
  rai::Transformation t;
  arr relPos;
  arr relPosJ;

  s1->rel.pos.write( std::cout );
  std::cout << std::endl;
  s2->rel.pos.write( std::cout );
  std::cout << std::endl;

  komo.world.kinematicsRelPos( relPos, relPosJ, s2->body, s2->rel.pos, s1->body, s1->rel.pos );
  t.pos = relPos;
  komo.setKinematicSwitch( time, true, "addRigid", s1->name, s2->name, t );

  /*auto * object1 = komo.world.getBodyByName( object1Name );
  auto * object2 = komo.world.getBodyByName( object2Name );

  for( auto s1 : object1->shapes )
  {
    for( auto s2 : object2->shapes )
    {
//      rai::Shape * object1 = komo.world.getShapeByName( s1->name );
//      rai::Shape * object2 = komo.world.getShapeByName( s2->name );
      rai::Transformation t;
      arr relPos;
      arr relPosJ;

      s1->rel.pos.write( std::cout );
      std::cout << std::endl;
      s2->rel.pos.write( std::cout );
      std::cout << std::endl;

      komo.world.kinematicsRelPos( relPos, relPosJ, s2->body, s2->rel.pos, s1->body, s1->rel.pos );

      t.pos = relPos;

      komo.setKinematicSwitch( time, true, "addRigid", s1->name, s2->name, t );
    }
  }*/
}


/*struct AllObjectsCollisionAvoidance:Feature
{
  AllObjectsCollisionAvoidance( double margin=.02 )
    : margin_( margin )
  {
  }

  virtual void phi( arr& y, arr& J, const rai::KinematicWorld& G, int t )
  {
    //::PairCollisionConstraint::phi( y, J, G, t );
//    arr tmp_y = zeros( 1 );

//    for( auto )
    uint dim = dim_phi( G );

    arr tmp_y = zeros( dim );
    arr tmp_J = zeros( dim, G.q.N );

    for( auto k = 0; k < G.proxies.N; ++k )
    {
      auto p = G.proxies( k );

      tmp_y( k ) = margin_ - p->d;

      rai::Shape *a = G.shapes(p->a);
      rai::Shape *b = G.shapes(p->b);

      auto arel=a->body->X.rot/(p->posA-a->body->X.pos);
      auto brel=b->body->X.rot/(p->posB-b->body->X.pos);

      arr posA;
      arr posB;
      arr JposA;
      arr JposB;
      G.kinematicsPos(posA, JposA, a->body, arel);
      G.kinematicsPos(posB, JposB, b->body, brel);

      double d1 = ( p->posA - p->posB ).length();
      double d2 = norm2( posA - posB );

      arr JnormD = Jnorm( posA - posB ) * ( JposA - JposB );

      tmp_J.setMatrixBlock( -JnormD, k, 0 );
      //tmp_J =

      //std::cout << p->d << " " << d1 << " " << d2 << std::endl;
    }

    // commit results
    y = tmp_y;
    if(&J) J = tmp_J;
  }

  rai::String shortTag(const rai::KinematicWorld& G){ return STRING("AllObjectsCOllisionAvoidance"); }

  uint dim_phi(const rai::KinematicWorld& G)
  {
    return ( G.shapes.N ) * ( G.shapes.N );
  }

private:
  double margin_;
};*/

//===========================================================================

void move_0(){

  KOMO komo;
  komo.setConfigFromFile();

  //komo.setSquaredFixJointVelocities();
  komo.setFixSwitchedObjects();
  komo.setSquaredQAccelerations();

  const double start_time = 1.0;

  //komo.setAlign( 1.0, 7, "container_1_front" );

  /////ACTIVE GET SIGHT CONTAINER 0
  {
    const double time = start_time + 0.0;

    komo.setTask( time, time + 1.0, new ActiveGetSight      ( "manhead",
                                                              "container_0",
                                                              ARR( -0.0, 0.2, 0.4 ),// pivot position  in container frame
                                                              ARR(  0.0, -1.0,0 )), // aiming dir
                  OT_sos, NoArr, 1e2 );
  }

  {
    const double time = start_time + 1.0;

//    komo.setTask( time, time + 1.0, new TakeView      ( ),
//                  OT_eq, NoArr, 1e2, 1 );

  }

  komo.setTask( 1.0, start_time + 8.0, new AxisAlignment( "container_0", ARR( 1.0, 0, 0 ) ), OT_eq, NoArr, 1e2 );
  komo.setTask( 1.0, start_time + 8.0, new OverPlaneConstraint ( komo.world,
                                                                 "container_0",
                                                                 "tableC",
                                                                 0.01
                                                                 ),  // pivot position  in container frame
                OT_ineq, NoArr, 1e2 );

  /////GRASP CONTAINER 0 ////
//  {
    const double time = start_time + 2.0 + 1.0;
    //arrive sideways
    //komo.setTask( time, time, new TM_Default( TMT_vec, komo.world, "handL", Vector_x ), OT_sos, {0.,0.,1.}, 1e1 );

    //disconnect object from table
    komo.setKinematicSwitch( time, true, "delete", "tableC", "container_0_bottom" );
    //connect graspRef with object
    komo.setKinematicSwitch( time, true, "ballZero", "handL", "container_0_left" );
    //komo.setKinematicSwitch( time, true, "addRigid", "handL", "container_1_handle", NoTransformation, NoTransformation );
//  }

  /////


//  /////ACTIVE GET SIGHT CONTAINER 1
//  {
//    const double time = start_time + 3.0;

//    komo.setTask( time, time + 1.0, new ActiveGetSight      ( "manhead",
//                                                              "container_1",
//                                                              //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
//                                                              ARR( -0.0, 0.2, 0.4 ) ),  // pivot position  in container frame
//                  OT_sos, NoArr, 1e2 );
//  }

  /////PLACE ON TABLE FOR CONTAINER 0
  {
  //  const double time = start_time + 4.5;

  //  komo.setPlace( time, "handL", "container_0_front", "tableL" );
  }

  /////HOMING
//    {
//      const double time = start_time + 5.0;

//      komo.setHoming(time, time+1, 1e-2); //gradient bug??
//    }

//  komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_right", "container_0_left", 0.05 ), OT_ineq, NoArr, 1e2 );
//  komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_front", "container_0_front", 0.05 ), OT_ineq, NoArr, 1e2 );
//  komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_right", "container_0_bottom", 0.05 ), OT_ineq, NoArr, 1e2 );
//  komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_bottom", "container_0_left", 0.05 ), OT_ineq, NoArr, 1e2 );


  komo.reset();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

void move_1(){

  KOMO komo;
  komo.setConfigFromFile();

  //komo.setSquaredFixJointVelocities();
  komo.setFixEffectiveJoints();
  komo.setFixSwitchedObjects();
  komo.setSquaredQAccelerations();

  // make container and target rigid
  //setRigid( 0.5, "container_1_bottom", "target", komo );
  //setRigid( 0.5, "container_1_bottom", "tableC", komo );


  // grasp container
  //komo.setGrasp( 1.0, "handL", "container_1_front" );
  //komo.setGrasp( 2.0, "handL", "target_1" ); // grasp ball

  //komo.setAlign( 1.0, 7, "container_1_front" );

  /////ACTIVE GET SIGHT CONTAINER 0  // 1->2
//  {
//    komo.setTask( 1.0, 2.0, new ActiveGetSight      ( "manhead",
//                                                              "container_0",
//                                                              ARR( -0.0, 0.2, 0.4 ) ),  // pivot position  in container frame
//                  OT_sos, NoArr, 1e2 );
//  }

  //////TAKE VIEW // 2->3
 /* {
//    komo.setTask( time, time + 1.0, new TakeView      ( ),
//                  OT_eq, NoArr, 1e2, 1 );

    auto *map = new TM_Transition(komo.world);
    map->posCoeff = 0.;
    map->velCoeff = 1.;
    map->accCoeff = 0.;
    komo.setTask( 2.0, 3.0, map, OT_sos, NoArr, 1e2, 1 );
  }

  //////OVER PLANE + AXIS ALIGNMENT
  //komo.setTask( 1.0, start_time + 8.0, new AxisAlignment( "container_1", ARR( 1.0, 0, 0 ) ), OT_eq, NoArr, 1e2 );
//  komo.setTask( 1.0, start_time + 8.0, new OverPlaneConstraint ( komo.world,
//                                                                 "container_1",
//                                                                 "tableC",
//                                                                 0.01
//                                                                 ),  // pivot position  in container frame
//                OT_ineq, NoArr, 1e2 );

  /////GRASP CONTAINER//// 3->4
//  {
    //arrive sideways
    //komo.setTask( time, time, new TM_Default( TMT_vec, komo.world, "handL", Vector_x ), OT_sos, {0.,0.,1.}, 1e1 );

    //disconnect object from table
    komo.setKinematicSwitch( 4.0, true, "delete", "tableC", "container_1_bottom" );
    //connect graspRef with object
    komo.setKinematicSwitch( 4.0, true, "ballZero", "handL", "container_1_left" );
    //komo.setKinematicSwitch( time, true, "addRigid", "handL", "container_1_handle", NoTransformation, NoTransformation );
//  }

  /////


  /////ACTIVE GET SIGHT CONTAINER 1 // 4->5
  {
    komo.setTask( 4.0, 5.0, new ActiveGetSight      ( "manhead",
                                                              "container_1",
                                                              //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
                                                              ARR( -0.0, 0.1, 0.4 ) ),  // pivot position  in container frame
                  OT_sos, NoArr, 1e2 );
  }

  /////TAKE VIEW CONTAINER 1 6->7
    {
      auto *map = new TM_Transition(komo.world);
      map->posCoeff = 0.;
      map->velCoeff = 1.;
      map->accCoeff = 0.;
      komo.setTask( 5.0, 6.0, map, OT_sos, NoArr, 1e2, 1 );
    }

  /////PLACE ON TABLE FOR CONTAINER 1
  {
    komo.setPlace( 7.0, "handL", "container_1_bottom", "tableC" );
  }*/
//  {
//     const double time = start_time + 5.0;

//    komo.setHoming(time, time+1, 1e-2); //gradient bug??
//  }

    //// GRASP BALL
    {
      //approach
      /*komo.setTask( 2.0, 2.4, new TM_Default(TMT_pos, komo.world, "handL" ), OT_sos, {0.,0.,-.15}, 1e1, 1);
      komo.setTask( 2.6, 3.0, new TM_Default(TMT_pos, komo.world, "handL" ), OT_sos, {0.,0.,.15},  1e1, 1);*/

//    komo.setTask(2.0, 2.5, new TM_Default(TMT_pos, komo.world, "handL"), OT_eq, {0.,0.,-.1}, 1e1, 1); //move down
//    komo.setTask(2.5, 3.0, new TM_Default(TMT_pos, komo.world, "handL"), OT_eq, {0.,0.,.1}, 1e1, 1); // move up

      // approach
      /*komo.setTask( 2.0, 2.25, new TM_Default(TMT_pos, komo.world, "handL", NoVector, "target", {0.,0.,0.45}), OT_sos, NoArr, 1e2);
      komo.setTask( 2.25, 5.0, new VerticalVelocity("handL", { 0.0,0.0 } ), OT_eq, NoArr, 1e1, 1 );
      */

      //disconnect object from container
      komo.setKinematicSwitch( 2.5/*8.0*/, true, "delete", NULL, "target" );
      //connect graspRef with object
      komo.setKinematicSwitch( 2.5/*8.0*/, true, "ballZero", "handL", "target" );
      // null velocity
      //komo.setTask( 2.5, 2.7, new TM_Default(TMT_pos, komo.world, "handL" ), OT_eq, {0.,0.,0.},  1e1, 1);

      // escape
      //komo.setTask( 2.7, 3.0, new TM_Default(TMT_pos, komo.world, "handL", NoVector, "container_1_bottom", {0.,0.,0.45}), OT_sos, NoArr, 1e2);

      // collision avoidance
      komo.setTask( 0.0, 9.0, new ApproxPointToShape( komo.world, "humanL", "container_1_front", 0.03 ), OT_ineq, NoArr, 1e2 );
      komo.setTask( 0.0, 9.0, new ApproxPointToShape( komo.world, "humanL", "container_1_left",  0.03  ),OT_ineq, NoArr, 1e2 );
      komo.setTask( 0.0, 9.0, new ApproxPointToShape( komo.world, "humanL", "container_1_right", 0.03 ), OT_ineq, NoArr, 1e2 );

      komo.setTask( 0.0, 9.0, new ApproxPointToShape( komo.world, "humanL", "tableC", 0.03 ), OT_ineq, NoArr, 1e2 );



      //komo.setTask( 2.7, 3.0, new VelocityDirection("handL", { 0.0,0.0,  1.0 } ), OT_eq, NoArr, 1e1, 0);

      //komo.setTask( 2.0, 2.25, new TM_Default(TMT_pos, komo.world, "handL", NoVector, "container_1_bottom", {0.,0.,0.5}), OT_sos, NoArr, 1e2);
      //komo.setTask( 2.6, 3.0, new TM_Default(TMT_pos, komo.world, "handL", NoVector, "container_1_bottom", {0.,0.,0.5}), OT_sos, NoArr, 1e2);

      /*komo.setTask( 2.25, 2.5, new TM_Default(TMT_pos, komo.world, "handL" ), OT_sos, {0.,0.,-.1}, 1e1, 1);

      komo.setTask( 2.5, 3.0, new TM_Default(TMT_pos, komo.world, "handL", NoVector, "target", {0.,0.,0.5}), OT_sos, NoArr, 1e2);
      komo.setTask( 2.5, 3.0, new TM_Default(TMT_pos, komo.world, "handL" ), OT_sos, {0.,0.,.1}, 1e1, 1);
      */

      //komo.setKinematicSwitch( time, true, "addRigid", "handR", "target", NoTransformation, NoTransformation );


      //komo.setPosition(time, time + 1.0, "handR", "target", OT_sos, NoArr, 1e2);
      //komo.setGrasp(time + 1.0, "handR", "target", 0, 1e1);
//      komo.setTask( 1.0, 9.0, new ShapePairCollisionConstraint( komo.world, "tableC", "handR", 0.05 ), OT_sos /*OT_ineq*/, NoArr, 1e2 );
//       komo.setTask( 1.0, 9.0, new ShapePairCollisionConstraint( komo.world, "container_1_front", "upWristL", 0.05 ), OT_ineq, NoArr, 1e2 );
//       komo.setTask( 1.0, 9.0, new ShapePairCollisionConstraint( komo.world, "container_1_left", "upWristL",  0.05  ),OT_ineq /*OT_ineq*/, NoArr, 1e2 );
//       komo.setTask( 1.0, 9.0, new ShapePairCollisionConstraint( komo.world, "container_1_right", "upWristL", 0.05 ), OT_ineq, NoArr, 1e2 );
//      komo.setTask( 1.0, 9.0, new ShapePairCollisionConstraint( komo.world, "container_1_bottom", "upWristL",0.05 ), OT_ineq /*OT_ineq*/, NoArr, 1e2 );

  }

    /////


  /////COLLISION AVOIDANCE
//  {
//    komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_right", "container_0_left", 0.05 ), OT_ineq, NoArr, 1e2 );
//    komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_front", "container_0_front", 0.05 ), OT_ineq, NoArr, 1e2 );
//    komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_right", "container_0_bottom", 0.05 ), OT_ineq, NoArr, 1e2 );
//    komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_bottom", "container_0_left", 0.05 ), OT_ineq, NoArr, 1e2 );

//      komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_front", "tableC", 0.05 ), OT_ineq, NoArr, 1e2 );

////    komo.setTask( start_time + 2.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_bottom", "tableCC" ), OT_ineq, NoArr, 1e2 );
//  }

//  if(komo.stepsPerPhase>2){ //velocities down and up
//    komo.setTask(time-.15, time, new TM_Default(TMT_pos, komo.world, "handL"), OT_sos, {0.,0.,-.1}, 1e1, 1); //move down
//    komo.setTask(time, time+.15, new TM_Default(TMT_pos, komo.world, "occluding_object_1"), OT_sos, {0.,0.,.1}, 1e1, 1); // move up
//  }

//  komo.setTask( 1.0, 2.0, new HeadGetSight( ARR(  1.0, -0.0, 1.9 ),    // object position
//                                            ARR(  1.0, -0.0, 1.9 ) ),  // pivot position
//                OT_sos, NoArr, 1e2 );

  //komo.setTask(.3, .5, new HandPositionMap(), OT_sos, ARR(.5,.5,1.3), 1e2);
  //komo.setTask(.8, 1., new HandPositionMap(), OT_sos, ARR(.8,0.,1.3), 1e2);
  //komo.setTask(.8, 1., new TM_Default(TMT_posDiff, komo.world, "/human/humanR", NoVector, "target", NoVector), OT_sos, NoArr, 1e2);

//  komo.setTask(.3, 1., new TM_Default(TMT_gazeAt, komo.world, "eyes", NoVector, "target", NoVector), OT_sos, NoArr, 1e2);

  //komo.setSlowAround( 5.0, .1, 1e3 );


  komo.reset();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.5, true);
}

//void move_pr2()
//{
//  rai::KinematicWorld kin;
//  kin.init( "model.g" );
//  kin.watch();
//  kin.write( std::cout );

//  rai::wait( 300, true );
//}

void move_debug(){

  {
    rai::KinematicWorld kin;
    kin.init( "debug-kin-parsing.kin" );
    kin.watch();
    kin.write( std::cout );
  }
  
  KOMO komo;
  //komo.setConfigFromFile();
  rai::KinematicWorld kin;
  kin.init( "debug-kin.kin" );
  kin.watch();

  komo.setModel( kin );

  // rai::Body *b = komo.world.getBodyByName("/human/base");
  // b->X.addRelativeTranslation(.3,0,0);

  //  komo.setHoming(-1., -1., 1e-1);
  //  komo.setSquaredQVelocities();
  /*komo.setSquaredFixJointVelocities();
  komo.setSquaredFixSwitchedObjects();
  komo.setSquaredQAccelerations();
  */

  komo.setTiming(2, 20, 5., 2/*, false*/);

  komo.setHoming(-1., -1., 1e-2); //gradient bug??
  komo.setSquaredQAccelerations();
 // komo.setSquaredFixJointVelocities(-1., -1., 1e3);
 // komo.setSquaredFixSwitchedObjects(-1., -1., 1e3);

  //komo.setPosition(1., 1.1, "humanL", "target", OT_sos, NoArr, 1e2);

  //komo.setPosition(1., 2.0, "manhead", "target", OT_sos, ARR(0,0,0), 1e2);  // objective to have the head on the target on this time slab
  //komo.setTask(startTime, endTime, new TM_Default(TMT_pos, world, shape, NoVector, shapeRel, NoVector), type, target, prec);

  //arr targetArr1 = HeadPoseMap::buildTarget( rai::Vector( 0, -0.3, 1.7 ), 80 );
  //komo.setTask(1.0, 3.0, new HeadPoseMap(), OT_sos, targetArr1, 1e2);

  komo.setGrasp( 1.0, "humanR", "occluding_object_1" );
//  komo.setTask( 1.0, 2.0, new HeadGetSight    ( ARR(  0.0, -1.0, 1.6 ),    // object position
//                                                ARR( -0.2, -0.6, 1.9 ) ),  // pivot position
//                                                OT_sos, NoArr, 1e2 );


//  komo.setTask( 1.0, 2.0, new HeadGetSight( ARR(  1.0, -0.0, 1.9 ),    // object position
//                                            ARR(  1.0, -0.0, 1.9 ) ),  // pivot position
//                OT_sos, NoArr, 1e2 );

  //komo.setTask(.3, .5, new HandPositionMap(), OT_sos, ARR(.5,.5,1.3), 1e2);
  //komo.setTask(.8, 1., new HandPositionMap(), OT_sos, ARR(.8,0.,1.3), 1e2);
  //komo.setTask(.8, 1., new TM_Default(TMT_posDiff, komo.world, "/human/humanR", NoVector, "target", NoVector), OT_sos, NoArr, 1e2);

//  komo.setTask(.3, 1., new TM_Default(TMT_gazeAt, komo.world, "eyes", NoVector, "target", NoVector), OT_sos, NoArr, 1e2);

  komo.setSlowAround( 3.0, .1, 1e3 );

  komo.reset();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

void move_blocks()
{
//  {
//    rai::KinematicWorld kin;
//    kin.init( "model_shelf.g" );
//    kin.watch();
//    kin.write( std::cout );

//    rai::wait( 30, true );
//  }

  KOMO komo;
  komo.setConfigFromFile();

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

  ///GRASP A, PUT IT on TABLE
  //grasp A
  komo.setKinematicSwitch( 3.0, true, "delete",   NULL, "block_r" );
  komo.setKinematicSwitch( 3.0, true, "ballZero", "handL", "block_r" );

  //place on table
  komo.setPlace( 4.0, "handL", "block_r", "tableC" );

  ///GRASP O, PUT IT on A
  //grasp
  komo.setKinematicSwitch( 5.0, true, "delete",   NULL, "block_o" );
  komo.setKinematicSwitch( 5.0, true, "ballZero", "handL", "block_o" );

  //put on A
  komo.setPlace( 6.0, "handL", "block_o", "block_r" );

  ///GRASP C, PUT IT on O
  //grasp
  komo.setKinematicSwitch( 7.0, true, "delete",   NULL, "block_b" );
  komo.setKinematicSwitch( 7.0, true, "ballZero", "handL", "block_b" );

  //put on O
  komo.setPlace( 8.0, "handL", "block_b", "block_o" );

  ///GRASP B, PUT IT on C
  //grasp
  komo.setKinematicSwitch( 9.0, true, "delete",   NULL, "block_g" );
  komo.setKinematicSwitch( 9.0, true, "ballZero", "handL", "block_g" );

  //put on B
  komo.setPlace( 10.0, "handL", "block_g", "block_b" );

  // launch komo
  komo.reset();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  //move_1();

  move_blocks();

  return 0;
}
