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

#include "geometric_levels.h"
#include "polgp_node.h"
#include "kin_equality_task.h"
#include "Kin/kin_swift.h"

static double eps() { return std::numeric_limits< double >::epsilon(); }
static double m_inf() { return -std::numeric_limits< double >::max(); }

// pose

PoseLevelType::PoseLevelType( POLGPNode * node, const KOMOFactory & komoFactory )
  : GeometricLevelBase( node, "pose", komoFactory )
{

}

void PoseLevelType::solve()
{
  //-- collect 'path nodes'
  //POLGPNodeL treepath = node_->getTreePath();

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < N_; ++w )
  {
    if( node_->bs()( w ) > eps() )
    {
      rai::KinematicWorld kin = node_->isRoot() ? *node_->startKinematics()( w ) : node_->parent()->effKinematics()( w );

      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      // debug
      auto cont_0 = kin.getBodyByName( "container_0" );
      auto cont_1 = kin.getBodyByName( "container_1" );

      CHECK( cont_0 != nullptr, "container_0" );
      CHECK( cont_1 != nullptr, "container_1" );

      //
      komo->setModel( kin, true, false, true, false, false );

//      setModel(
//            W,
//            useSwift
//            rai::getParameter<bool>("KOMO/meldFixedJoints", false),
//            rai::getParameter<bool>("KOMO/makeConvexHulls", true),
//            rai::getParameter<bool>("KOMO/makeSSBoxes", false),
//            rai::getParameter<bool>("KOMO/activateAllContact", false)
//            );

      komo->setTiming( 1., 2, 5., 1/*, true*/ );
      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??
      komo->setSquaredQVelocities();
      komo->setFixSwitchedObjects(-1., -1., 1e3);

      komo->groundTasks( 0., *node_->folStates()( w ) );

      komo->reset();
      try{
        komo->run();
      } catch( const char* msg ){
        cout << "KOMO FAILED: " << msg <<endl;
      }

      // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
      //komo->displayTrajectory();
      COUNT_evals += komo->opt->newton.evals;
      COUNT_kin += rai::KinematicWorld::setJointStateCount;
      COUNT_poseOpt++;
      //poseCount++;

      // save results
//      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )

      Graph result = komo->getReport();
//      DEBUG( FILE( "z.problem.cost" ) << result; )
      double cost = result.get<double>( { "total","sqrCosts" } );
      double constraints = result.get<double>( { "total","constraints" } );

      //
      //std::cout << "Pose problem " << node_->id() << " solved with :" << cost << " " << constraints << std::endl;
      //

      if( ! node_->isRoot() )
      {
        cost += node_->parent()->poseGeometricLevel()->cost( w );
      }

      // if this pose leads to the smaller cost so far
      if( ! solved_( w ) || cost < costs_( w ) )
      {
        bool solved = constraints < maxConstraints_ && cost < maxCost_;

        if( ! solved )
        {
          std::cout << "!!!can't be solved for:" << node_->id() << " cost:" << cost << std::endl;
        }
//        else
//        {
//          std::cout << "ok for:" << node_->id() << " cost:" << cost << std::endl;
//        }

        costs_( w ) = cost;
        constraints_( w ) = constraints;
        solved_( w )    = solved;
        infeasibles_( w ) = ! solved;
        komos_( w ) = komo;

        // update effective kinematic
        node_->effKinematics()( w ) = *komos_( w )->configurations.last();

        // update switch
        for( rai::KinematicSwitch *sw: komos_( w )->switches )
        {
          //    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
          if( sw->timeOfApplication>=2 ) sw->apply( node_->effKinematics()( w ) );
        }
        node_->effKinematics()( w ).topSort();
        //DEBUG( node->effKinematics()( w ).checkConsistency(); )
        node_->effKinematics()( w ).getJointState();
      }
    }
  }

  backtrack();
}

void PoseLevelType::backtrack()
{
  if( node_->isSymbolicallyTerminal() )
  {
    // if the node is logically terminal and if a pose has been found for each world,
    // then the node is considered as pose-solve and pose-terminal

    bool solved = true;
    bool infeasible = false;
    // test if it is solved for all worlds
    for( auto w = 0; w < N_; ++w )
    {
      if( node_->bs()( w ) > eps() )
      {
        solved = solved && solved_( w );
        infeasible = infeasible || infeasibles_( w );
      }
    }

    // commit results
    isSolved_     = solved;
    isInfeasible_ = infeasible;

    // if it is solved, we compute the expected cost
    if( isSolved_ )
    {
      double expectedCost = 0;
      for( auto w = 0; w < N_; ++w )
      {
        if( node_->bs()( w ) > eps() )
        {
          expectedCost += node_->bs()( w ) * costs_( w );
        }
      }

      isTerminal_      = true;
    }

    if( isInfeasible_ )
    {
      node_->labelInfeasible(); // label this sequence of actions as infeasible
    }
//    if( isSolved_ )
//    {
//      std::cout << "Terminal node: " << node_->id() << " set solved" << std::endl;
//    }
  }
  else
  {
    // if the node is not terminal, we set it as solve, if the pose optimization was successful and the children of its best family are solved
    bool solved = true;
    bool infeasible = false;

    for( auto w = 0; w < N_; ++w )
    {
      if( node_->bs()( w ) > eps() )
      {
        solved = solved && solved_( w );
        infeasible = infeasible || infeasibles_( w );
      }
    }

    for( auto s : node_->bestFamily() )
    {
      solved = solved && s->poseGeometricLevel()->isSolved();
      infeasible = infeasible || s->poseGeometricLevel()->isInfeasible();
    }

    // commit result
    isSolved_ = solved;
    isInfeasible_ = infeasible;

    if( isInfeasible_ )
    {
      node_->labelInfeasible();
    }

//    if( isSolved_ )
//    {
//      std::cout << node_->id() << " set solved" << std::endl;
//    }
  }

  // continue backtracking
  if( node_->parent() )
  {
    node_->parent()->poseGeometricLevel()->backtrack();
  }
}


// path

PathLevelType::PathLevelType( POLGPNode * node, const KOMOFactory & komoFactory, uint microSteps )
  : GeometricLevelBase( node, "path", komoFactory )
  , microSteps_( microSteps )
{

}

void PathLevelType::solve()
{
  //-- collect 'path nodes'
  POLGPNodeL treepath = node_->getTreePath();

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < N_; ++w )
  {
    if( node_->bs()( w ) > eps() )
    {
      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      komo->setModel( *node_->startKinematics()( w ), true, false, true, false, false );
      komo->setTiming( start_offset_ + node_->time() + end_offset_, microSteps_, 5., 2/*, true*/ );

      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??

      komo->setFixEffectiveJoints();
      komo->setFixSwitchedObjects();
      komo->setSquaredQAccelerations();
      //komo->setSquaredFixJointVelocities();// -1., -1., 1e3 );
      //komo->setSquaredFixSwitchedObjects();// -1., -1., 1e3 );

      for( auto node:treepath )
      {
        auto time = ( node->parent() ? node->parent()->time(): 0. );   // get parent time
        komo->groundTasks( start_offset_ + time, *node->folStates()( w ) );          // ground parent action (included in the initial state)
      }

//      DEBUG( FILE("z.fol") <<fol; )
//          DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
      komo->reset();
      try{
        komo->run();
      } catch(const char* msg){
        cout << "KOMO FAILED: " << msg <<endl;
      }

      // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
      //komo->displayTrajectory();
      COUNT_evals += komo->opt->newton.evals;
      COUNT_kin += rai::KinematicWorld::setJointStateCount;
      COUNT_pathOpt++;

//      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
      //komo->checkGradients();

      Graph result = komo->getReport();
      //DEBUG( FILE("z.problem.cost") << result; )
      double cost = result.get<double>({"total","sqrCosts"});
      double constraints = result.get<double>({"total","constraints"});

      if( ! costs_( w ) || cost < costs_( w ) )     //
      {
        bool solved = true;//constraints < maxConstraints_;
        //bool solved = constraints < maxConstraints_;

        costs_( w )       = cost;                     //
        constraints_( w ) = constraints;              //
        solved_( w )      = solved;
        infeasibles_( w ) = ! solved;                   //
        komos_( w )       = komo;                     //

        // back the best komo for this world
        for( POLGPNode * node = node_; node; node = node->parent() )
        {
          node->pathGeometricLevel()->komo( w ) = komo;
        }
      }
    }
  }

  backtrack();
}

void PathLevelType::backtrack()
{
  if( node_->isSymbolicallyTerminal() )
  {
    // if the node is logically terminal and if a pose has been found for each world,
    // then the node is considered as pose-solve and pose-terminal

    bool solved = true;
    bool infeasible = false;
    // test if it is solved for all worlds
    for( auto w = 0; w < N_; ++w )
    {
      if( node_->bs()( w ) > eps() )
      {
        solved = solved && solved_( w );
        infeasible = infeasible || infeasibles_( w );
      }
    }

    // commit results
    isSolved_ = solved;
    isInfeasible_ = infeasible;

    //
    if( isSolved_ )
    {
      isTerminal_      = true;
    }

    if( infeasible )
    {
      node_->labelInfeasible();
    }
  }
  else
  {
    // if the node is not terminal, we set it as solve, if the pose optimization was successful and the children of its best family are solved
    bool solved = true;
    bool infeasible = false;

    for( auto s : node_->bestFamily() )
    {
      solved = solved && s->pathGeometricLevel()->isSolved();
      infeasible = infeasible || s->pathGeometricLevel()->isInfeasible();
    }

    // commit results
    isSolved_ = solved;
    isInfeasible_ = infeasible;

    if( isInfeasible_ )
    {
      node_->labelInfeasible();
    }
  }

  // continue backtracking
  if( node_->parent() )
  {
    node_->parent()->pathGeometricLevel()->backtrack();
  }
}

// joint path

JointPathLevelType::JointPathLevelType( POLGPNode * node, const KOMOFactory & komoFactory, uint microSteps )
  : GeometricLevelBase( node, "joint path", komoFactory )
  , microSteps_( microSteps )
{

}

void JointPathLevelType::solve()
{
  //-- collect 'path nodes'
  POLGPNodeL treepath = node_->getTreePath();

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < N_; ++w )
  {
    if( node_->bs()( w ) > eps() )
    {
      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      komo->setModel( *node_->startKinematics()( w ), true, false, true, false, false );
      komo->setTiming( start_offset_ + node_->time() + end_offset_, microSteps_, 5., 2/*, true*/ );

      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??

      komo->setFixEffectiveJoints();
      komo->setFixSwitchedObjects();
      komo->setSquaredQAccelerations();
      //komo->setSquaredFixJointVelocities( -1., -1., 1e3 );
      //komo->setSquaredFixSwitchedObjects( -1., -1., 1e3 );

      for( auto node:treepath )
      {
        // set task
        auto time = ( node->parent() ? node->parent()->time(): 0. );   // get parent time

        komo->groundTasks( start_offset_ +  time, *node->folStates()( w ), 1 );          // ground parent action (included in the initial state)

        if( node->time() > 0 )
        {
          uint stepsPerPhase = node_->pathGeometricLevel()->komo( w )->stepsPerPhase; // get number of steps per phases
          uint nodeSlice = stepsPerPhase * ( start_offset_ + node->time() ) - 1;
          arr q = zeros( node_->pathGeometricLevel()->komo( w )->configurations( nodeSlice )->q.N );

          // set constraints enforcing the path equality among worlds
          int nSupport = 0;
          for( auto x = 0; x < N_; ++x )
          {
            if( node->bs()( x ) > 0 )
            {
              auto komo = node->pathGeometricLevel()->komo( x );

              CHECK( node->pathGeometricLevel()->komo( x )->configurations.N > 0, "one node along the solution path doesn't have a path solution already!" );

              q += node->bs()( x ) * node->pathGeometricLevel()->komo( x )->configurations( nodeSlice )->q;

              nSupport++;
            }
          }

          if( nSupport > 1 )  // enforce kin equality between at least two worlds, useless with just one world!
          {
            AgentKinEquality * task = new AgentKinEquality( node->id(), q );  // tmp camille, think to delete it, or komo does it?
            double slice_t = start_offset_ + node->time() - 1.0 / stepsPerPhase;
            komo->setTask( slice_t, slice_t, task, OT_eq, NoArr, 1e2  );

            //
            //std::cout << slice_t << "->" << slice_t << ": kin equality " << std::endl;
            //
          }
        }
      }

//      DEBUG( FILE("z.fol") <<fol; )
//      DEBUG( komo->MP->reportFeatures( true, FILE("z.problem") ); )

      komo->reset();
      try{
        komo->run();
      } catch(const char* msg){
        cout << "KOMO FAILED: " << msg <<endl;
      }

      // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
      //komo->displayTrajectory();
      COUNT_evals += komo->opt->newton.evals;
      COUNT_kin += rai::KinematicWorld::setJointStateCount;
      //COUNT_jointPathOpt++;

//      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
//      komo->checkGradients();

      Graph result = komo->getReport();
      //DEBUG( FILE("z.problem.cost") << result; )
      double cost = result.get<double>({"total","sqrCosts"});
      double constraints = result.get<double>({"total","constraints"});

      if( ! costs_( w ) || cost < costs_( w ) )       //
      {
        bool solved = true; //constraints < maxConstraints_;

        costs_( w )       = cost;                     //
        constraints_( w ) = constraints;              //
        solved_( w )      = solved;         //
        infeasibles_( w ) = ! solved;         //
        komos_( w )       = komo;
      }
    }
  }

  backtrack();
}

void JointPathLevelType::backtrack()
{
  if( node_->isSymbolicallyTerminal() )
  {
    // if the node is logically terminal and if a pose has been found for each world,
    // then the node is considered as pose-solve and pose-terminal

    bool solved = true;
    bool infeasible = false;
    // test if it is solved for all worlds
    for( auto w = 0; w < N_; ++w )
    {
      if( node_->bs()( w ) > eps() )
      {
        solved = solved && solved_( w );
        infeasible = infeasible || infeasibles_( w );
      }
    }

    isSolved_ = solved;
    isInfeasible_ = infeasible;

    if( isSolved_ )
    {
      isTerminal_      = true;
    }
  }
  else
  {
    // if the node is not terminal, we set it as solve, if the pose optimization was successful and the children of its best family are solved
    bool solved = true;
    bool infeasible = false;

    for( auto s : node_->bestFamily() )
    {
      solved = solved && s->jointPathGeometricLevel()->isSolved();
      infeasible = infeasible || s->jointPathGeometricLevel()->isInfeasible();
    }

    isSolved_ = solved;
    isInfeasible_ = infeasible;
  }

  // continue backtracking
  if( node_->parent() )
  {
    node_->parent()->jointPathGeometricLevel()->backtrack();
  }
}
