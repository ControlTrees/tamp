#include "ao_search.h"

#include <list>

#include <Core/util.tpp>

#include <KOMO/komo.h>
//#include <Motion/motion.h>
#include <Kin/taskMaps.h>

//#include <LGP/LGP.h>

#include "node_visitors.h"

//=====================free functions======================
static double eps() { return std::numeric_limits< double >::epsilon(); }

static void generatePngImage( const std::string & name )
{
  std::string nameCopy( name );
  const std::string ext( ".gv" );
  std::string newName = nameCopy.replace( nameCopy.find( ext ), ext.length(), ".png" );

  std::stringstream ss;
  ss << "dot"   << " ";
  ss << "-Tpng" << " ";
  ss << "-o"    << " ";
  ss << newName << " ";
  ss << name;

  system( ss.str().c_str() );
}

//===========================================================================
AOSearch::AOSearch( const KOMOFactory & komoFactory )
  : komoFactory_( komoFactory )
  , alternativeNumber_( 0 )
  , currentPolicyFringeInitialized_( false )
{

}

// modifiers
void AOSearch::prepareFol( const std::string & folDescription )
{
  const rai::String notObservableTag = "NOT_OBSERVABLE";

  Graph KB;
  KB.read( FILE( folDescription.c_str() ) );

  // fully observable case
  if( KB[ beliefStateTag_ ] == nullptr )
  {
    // create dummy array
    folWorlds_ = rai::Array< std::shared_ptr<FOL_World> > ( 1 );
    std::shared_ptr<FOL_World> fol = std::make_shared<FOL_World>();
    fol->init(FILE(folDescription.c_str()));
    folWorlds_( 0 ) = fol;
    fol->reset_state();
    // create dummy bs in observable case
    bs_ = arr( 1 );
    bs_( 0 ) = 1.0;
  }
  // partially observable case
  else
  {
    // get number of possible worlds
    auto bsGraph = &KB.get<Graph>( beliefStateTag_ );
    const uint nWorlds = bsGraph->d0;

    // generate all the possible fol
    folWorlds_ = rai::Array< std::shared_ptr<FOL_World> > ( nWorlds );
    bs_ = arr( nWorlds );
    for( uint w = 0; w < nWorlds; w++ )
    {
      // retrieve the facts of the belief state
      std::shared_ptr<FOL_World> fol = std::make_shared<FOL_World>();
      fol->init(FILE(folDescription.c_str()));
      auto n = bsGraph->elem(w);

      StringA fact;
      // add fact
      for( auto s : n->parents ) fact.append( s->keys.last() );
      //fol->addFact(fact);

      // tag this fact as not observable
      StringA notObservableFact; notObservableFact.append( notObservableTag );
      for( auto s : fact ) notObservableFact.append( s );

      fol->addFact(notObservableFact);
      fol->reset_state();

      //std::cout << *fol << std::endl; // tmp
      folWorlds_(w) = fol;
      bs_(w) = n->get<double>();
    }

    // check that the belief state sums to 1
    double total = 0;
    for( auto p : bs_ ) total += p;

    CHECK( total == 1.00, "wrong belief state definition, the total of the probabilities doesn't sum to 1" );
  }
}

void AOSearch::prepareKin( const std::string & kinDescription )
{
  Graph G( kinDescription.c_str() );

  if( G[ beliefStateTag_ ] == nullptr )
  {
    auto kin = std::make_shared< rai::KinematicWorld >();
    kin->init( kinDescription.c_str() );
    computeMeshNormals( kin->shapes );
    kin->calc_fwdPropagateFrames();
    //kin->watch(/*true*/);

    kinematics_.append( kin );
  }
  else
  {
    auto bsGraph = &G.get<Graph>( beliefStateTag_ );
    const uint nWorlds = bsGraph->d0;

    // build the different worlds
    for( uint w = 0; w < nWorlds; w++ )
    {
      Graph kinG( kinDescription.c_str() );

      // copy unobservable facts
      auto n = bsGraph->elem(w);

      for( auto nn : n->graph() )
      {
        nn->newClone( kinG );
      }

      auto kin = std::make_shared< rai::KinematicWorld >();
      kin->init( kinG );
      computeMeshNormals( kin->shapes );
      kin->calc_fwdPropagateFrames();
      //
      //kin->watch(/*true*/);
      //
      kinematics_.append( kin );
    }
  }
}

void AOSearch::prepareTree()
{
  CHECK( folWorlds_.d0 == kinematics_.d0, "There should be as many logic worlds as kinematic worlds!, check the fol and kin description files!" );

  root_ = new POLGPNode( folWorlds_, kinematics_, bs_, komoFactory_ );
}

void AOSearch::prepareDisplay()
{
  for( auto w = 0; w < folWorlds_.d0; ++w )
  {
    std::string namePose = std::string( "pose" ) + std::string( "-world-" ) + std::to_string( w );
    std::string nameSeq =  std::string( "seq" ) + std::string( "-world-" )  + std::to_string( w );
    std::string namePath = std::string( "path" ) + std::string( "-world-" ) + std::to_string( w );

    poseViews_.append( std::make_shared< OrsPathViewer >( namePose.c_str(),  1, -0   ) );
    pathViews_.append( std::make_shared< OrsPathViewer >( namePath.c_str(), .1, -1   ) );
  }

  threadOpenModules( true );
}

//void AOSearch::registerGeometricLevel( GeometricLevelFactoryBase::ptr const& factory )
//{
//  geometricLevelFactories_.append( factory );
//}

void AOSearch::solveSymbolically()
{
  std::cout << "AOSearch::solveSymbolically" << std::endl;

  auto s = 0;
  while( ! isSymbolicallySolved() )
  {
    s++;
    auto nodes = getNodesToExpand();

    for( auto node : nodes )
    {
      // expand
      node->expand();

      // generate rollouts for each child
      for( auto f : node->families() )
      {
        for( auto c : f )
        {
          c->generateMCRollouts( 50, 10 );
        }
      }

      {
      // save the current state of the search
      //std::stringstream namess;
      //namess << "exploration-" << s << ".gv";
      //printSearchTree( namess.str() );
      }

      // backtrack result
      node->backTrackBestExpectedPolicy();
    }
  }

  PrintRewardsVisitor printer;
  root_->acceptVisitor( printer );
}

void AOSearch::generateAlternativeSymbolicPolicy()
{
  std::cout << "AOSearch::generateAlternativeSymbolicPolicy" << std::endl;

  // gather the current policy fringe if it has not been gathered
  if( ! currentPolicyFringeInitialized_ )
  {
    CHECK( currentPolicyFringe_.size() == 0, "currentPolicyFringe_ corrupted!" );

    utility::gatherPolicyFringe( root_, currentPolicyFringe_ );
    currentPolicyFringeInitialized_ = true;
  }

  if( currentPolicyFringe_.size() > 0 )
  {
    // debug log
    for( auto f : currentPolicyFringe_ )
    {
      std::cout << "alternative family:" << std::endl;

      for( auto c : f )
      {
        std::cout << "alternative node to expand:" << c->id() << ":" << c->expecteTotalReward() << std::endl;
      }
      //std::cout << "alternative node to expand:" << alternativeNode->id() << ":" << alternativeNode->expecteTotalReward() << std::endl;
    }

    //////here define a strategy to choose the action to change
    // for the moment just take the last one
    auto alternativeFamily = *std::prev( currentPolicyFringe_.end() );
    //////

    // solve alternative family
    uint s = 0;
    for( auto alternativeNode : alternativeFamily )
    {
      while( ! alternativeNode->isSymbolicallySolved() )
      {
        s++;

        auto nodes = getNodesToExpand( alternativeNode );

        for( auto node : nodes )
        {
          // expand
          node->expand();

          // generate rollouts for each child
          for( auto f : node->families() )
          {
            for( auto c : f )
            {
              c->generateMCRollouts( 50, 10 );
            }
          }

//          {
//            // save the current state of the search
//            std::stringstream namess;
//            namess << "exploration-alternative-" << alternativeNumber_ << "-" << s << ".gv";
//            printSearchTree( namess.str() );
//          }

          // backtrack result
          node->backTrackBestExpectedPolicy( alternativeNode );
        }
      }
    }

    // backup old policy
    currentPolicyFringe_.erase( alternativeFamily );
    alternativeStartNode_ = alternativeFamily( 0 )->parent();
    nextFamilyBackup_     = alternativeStartNode_->bestFamily();
    currentPolicyFringeBackup_ = currentPolicyFringe_;
    alternativeStartNode_->setBestFamily( alternativeFamily );
    currentPolicyFringe_.clear();
    currentPolicyFringeInitialized_ = false;
    //

    alternativeNumber_ ++;
  }
}

void AOSearch::revertToPreviousPolicy()
{
  CHECK( alternativeStartNode_ !=  nullptr, "the backed up policy is invalid" );
  CHECK( nextFamilyBackup_.d0 > 0, "the backed up policy is invalid" );
  CHECK( nextFamilyBackup_( 0 )->parent() ==  alternativeStartNode_, "the backed up policy is invalid" );

  currentPolicyFringe_ = currentPolicyFringeBackup_;
  currentPolicyFringeInitialized_ = true;
  alternativeStartNode_->setBestFamily( nextFamilyBackup_ );
}

void AOSearch::solveGeometrically()
{
  CHECK( isSymbolicallySolved(), "try to optimize geometrically although no symbolic solution has been found!" );

  /// POSE OPTIMIZATION
  optimizePoses();      // optimizes poses of the current best solution

  if( isPoseSolved() )
  {
    /// PATH OPTIMIZATION
    optimizePaths();      // optimizes paths of the current best solution

    if( isPathSolved() )
    {
      /// JOINT PATH OPTIMIZATION
      optimizeJointPaths();   // optimizes joint paths of the current best solution
    }
  }

}

/*void AOSearch::addMcRollouts()
{
  std::cout << "AOSearch::addMcRollouts" << std::endl;

  resetSolvedStatusFrom( root_ );

  addMcRolloutsFrom( root_ );

  PrintRewardsVisitor printer;
  root_->acceptVisitor( printer );

  if( ! isSymbolicallySolved() )
  {
    solveSymbolically();
  }
}*/

/*void AOSearch::addMcRolloutsFrom( POLGPNode * node )
{
  for( auto f : node->families() )
  {
    for( auto c : node->bestFamily() )
    {
      c->generateMCRollouts( 50, 10 );
    }
  }

  node->backTrackBestExpectedPolicy();

  for( auto c : node->bestFamily() )
  {
    addMcRolloutsFrom( c );
  }
}*/

void AOSearch::optimizePoses()
{
  optimizePosesFrom( root_ );
}

void AOSearch::optimizePaths()
{
  auto nodes = getTerminalNodes();

  for( auto n : nodes )
  {
    n->solvePathProblem();
  }
}

void AOSearch::optimizeJointPaths()
{
  auto nodes = getTerminalNodes();

  for( auto n : nodes )
  {
    n->solveJointPathProblem();
  }
}

void AOSearch::optimizePosesFrom( POLGPNode * node )
{
  //std::cout << "solve pose problem for:" << node->id() << std::endl;
  node->solvePoseProblem();

  for( auto c : node->bestFamily() )
  {
    optimizePosesFrom( c );
  }
}

tmp::Policy::ptr AOSearch::getPolicy() const
{
  tmp::PolicyBuilder builder( root_ );
  return builder.getPolicy();
}

/*void AOSearch::resetSolvedStatusFrom( POLGPNode * node )
{
  if( ! node->isSymbolicallyTerminal() && node->isSymbolicallySolved() )
  {
    node->resetSymbolicallySolved();

    for( auto c : node->bestFamily() )
    {
      resetSolvedStatusFrom( c );
    }
  }
}*/

void AOSearch::updateDisplay( const WorldID & ww, bool poses, bool seqs, bool paths )
{
  std::list< std::size_t > worldIds;

  if( ww.id() == -1 )
  {
    for ( auto w = 0; w < folWorlds_.d0; ++w )
    {
      worldIds.push_back( w );
    }
  }
  else
  {
    worldIds.push_back( ww.id() );
  }

  for( auto w : worldIds )
  {
    // get the terminal node for the world w, in the case of stochaticity
    POLGPNode * node = getTerminalNode( WorldID( w ) );

    if( poses && node->komoPoseProblems()( w ) && node->komoPoseProblems()( w )->configurations.N )
      poseViews_( w )->setConfigurations( node->komoPoseProblems()( w )->configurations );
    else poseViews_( w )->clear();

    if( paths && node->komoPathProblems()( w ) && node->komoPathProblems()( w )->configurations.N )
      pathViews_( w )->setConfigurations( node->komoPathProblems()( w )->configurations );
    else pathViews_( w )->clear();

//    if( paths && node->komoJointPathProblems()( w ) && node->komoJointPathProblems()( w )->MP->configurations.N )
//      pathViews_( w )->setConfigurations( node->komoJointPathProblems()( w )->MP->configurations );
//    else pathViews_( w )->clear();
  }
}

rai::Array< POLGPNode * > AOSearch::getNodesToExpand() const
{
  return getNodesToExpand( root_ );
}

rai::Array< POLGPNode * > AOSearch::getNodesToExpand( POLGPNode * node ) const
{
  rai::Array< POLGPNode * >  nodes;

  // starts from root
  if( ! node->isSymbolicallySolved() )
  {
    if( ! node->isExpanded() )
    {
      nodes.append( node );
    }
    else
    {
      for( auto c : node->bestFamily() )
      {
//        if( ! c->isExpanded() ) // this part of code can be safely commented because it does n'tbring anything ( the recursiondoes it anyway )
//        {
//          nodes.append( c );
//        }
//        else
//        {
          nodes.append( getNodesToExpand( c ) );
//        }
      }
    }
  }

  return nodes;
}


void AOSearch::printPolicy( const std::string & name, bool generatePng ) const
{
  std::stringstream ss;
  printPolicy( ss );

  std::ofstream fs;
  fs.open( name );
  fs << ss.str();
  fs.close();

  if( generatePng )
  {
    generatePngImage( name );
  }
}

void AOSearch::printSearchTree( const std::string & name, bool generatePng ) const
{
  std::stringstream ss;
  printSearchTree( ss );

  std::ofstream fs;
  fs.open( name );
  fs << ss.str();
  fs.close();

  if( generatePng )
  {
    generatePngImage( name );
  }
}

void AOSearch::printPolicy( std::iostream & ss ) const
{
  ss << "digraph g{" << std::endl;

  printPolicy( root_, ss );

  ss << "}" << std::endl;
}

void AOSearch::printSearchTree( std::iostream & ss ) const
{
  ss << "digraph g{" << std::endl;

  printSearchTree( root_, ss );

  ss << "}" << std::endl;
}

//--------private----------------//

rai::Array< POLGPNode * > AOSearch::getTerminalNodes() const
{
  return getTerminalNodes( root_ );
}

rai::Array< POLGPNode * > AOSearch::getTerminalNodes( POLGPNode * n ) const
{
  rai::Array< POLGPNode * > nodes;

  if( n->isSymbolicallyTerminal() )
  {
    nodes.append( n );
  }
  else
  {
    for( auto c : n->bestFamily() )
    {
      nodes.append( getTerminalNodes( c ) );
    }
  }

  return nodes;
}

POLGPNode * AOSearch::getTerminalNode( const WorldID & w ) const
{
  // could be more generale and return a list of node in case of stochastic world
  return utility::getTerminalNode( root_, w );
}

void AOSearch::printPolicy( POLGPNode * node, std::iostream & ss ) const
{
  for( auto c : node->bestFamily() )
  {
    std::stringstream ss1;
    ss1 << node->bestActionStr();

    auto diffFacts = c->differentiatingFacts();

    for( auto fact : c->differentiatingFacts() )
      ss1 << std::endl << fact;

    if( node->bestFamily().N > 1 )
    {
      ss1 << std::endl << "p=" << c->pHistory();
      ss1 << std::endl << "q=" << c->pHistory() / node->pHistory();
    }

    auto label = ss1.str();

    ss << node->id() << "->" << c->id() << " [ label=\"" << label << "\" ]" << ";" << std::endl;

    printPolicy( c, ss );
  }
}

void AOSearch::printSearchTree( POLGPNode * node, std::iostream & ss ) const
{
  for( auto f : node->families() )
  {
    for( auto c : f )
    {
      std::stringstream ss1;
      //ss1 << node->bestActionStr();
      ss1 << c->leadingActionStr();

      auto diffFacts = c->differentiatingFacts();

      for( auto fact : c->differentiatingFacts() )
        ss1 << std::endl << fact;

      //if( node->bestFamily().N > 0 )
      //{
      ss1 << std::endl << "p=" << c->pHistory();
      ss1 << std::endl << "q=" << c->pHistory() / node->pHistory();
      ss1 << std::endl << "g=" << c->prefixReward();
      ss1 << std::endl << "h=" << c->expecteFutureReward();
      //}

      auto label = ss1.str();

      ss << node->id() << "->" << c->id() << " [ label=\"" << label << "\" ]" << ";" << std::endl;

      printSearchTree( c, ss );
    }
  }
}


//===========================================================================
