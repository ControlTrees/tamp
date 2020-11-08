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

#include <iterative_deepening.h>
#include <policy_builder.h>

#include <chrono>

namespace tp
{

IterativeDeepeningPlanner::IterativeDeepeningPlanner()
  : dmax_( 15 )
{

}

void IterativeDeepeningPlanner::setFol( const std::string & folDescription )
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

      std::cout << "n:" << *n << std::endl;

      // add facts
      double probability = -1;

      for( auto nn : n->graph() )
      {
        StringA fact;

        for( auto f : nn->parents )
        {
          fact.append( f->keys.last() );
        }

        if( ! fact.empty() )
        {
          // tag this fact as not observable
          StringA notObservableFact; notObservableFact.append( notObservableTag );
          for( auto s : fact ) notObservableFact.append( s );

          fol->addFact(notObservableFact);

          //std::cout << "fact:" << fact << std::endl;
          //std::cout << "notObservableFact:" << notObservableFact << std::endl;
        }
        else
        {
          probability = nn->get<double>();
          //std::cout << probability << std::endl;
        }
      }

      fol->reset_state();

      //std::cout << *fol << std::endl; // tmp
      folWorlds_(w) = fol;
      bs_(w) = probability;
    }

    // check that the belief state sums to 1
    double total = 0;
    for( auto p : bs_ ) total += p;

    CHECK( total == 1.00, "wrong belief state definition, the total of the probabilities doesn't sum to 1" );
  }

  root_ = std::make_shared< PONode >( folWorlds_, bs_ );
}

void IterativeDeepeningPlanner::solve()
{
  std::cout << "IterativeDeepeningPlanner::solveSymbolically" << std::endl;

  solveIterativeDepthFirst();
  //solveBreadthFirst();
}

void IterativeDeepeningPlanner::integrate( const Policy::ptr & policy )
{ 

}

Policy::ptr IterativeDeepeningPlanner::getPolicy() const
{
  Policy::ptr policy;

//  if( solutions_.size() > 0 )
//  {
//    policy = *solutions_.begin();
//  }

  return policy;
}

MotionPlanningOrder IterativeDeepeningPlanner::getPlanningOrder() const
{
  MotionPlanningOrder po( getPolicy()->id() );

  //
  po.setParam( "type", "jointPath" );
  //

  return po;
}



void IterativeDeepeningPlanner::solveIterativeDepthFirst()
{
  std::cout << "Depth First .." << std::endl;

  for( uint d = 1; d < dmax_; ++d )
  {
    std::cout << "depth = " << d << std::endl;

    iterativeDepthFirstExpand( root_, d );

    extractSolutions();

    std::cout << "expanded :" << n_exp_ << std::endl;
  }
}

PONode::ptr IterativeDeepeningPlanner::iterativeDepthFirstExpand( const PONode::ptr & node, uint e )
{
  if( e == 0 && node->isTerminal() )
  {
    return node;
  }
  else if( e > 0 )
  {
    if( ! node->isExpanded() )
    {
      n_exp_++;

      auto start = std::chrono::high_resolution_clock::now();

      n_exp_++;
      node->expand();

      auto elapsed = std::chrono::high_resolution_clock::now() - start;
      long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

//      if( n_exp_ % 100 == 0 )
//        std::cout << "expansion time (ms):" << ms << " n families:" << node->families().size() << std::endl;
    }

    for( auto f : node->families() )
    {
      for( auto c : f )
      {
        auto found = iterativeDepthFirstExpand( c, e - 1 );

        if( found )
        {
          std::cout << "leaf for bs:" << found->bs() << std::endl;

          found->backTrackSolveStatus();

          std::cout << "root solved:" << root_->isSolved() << std::endl;


          terminals_.push_back( found );
        }
      }
    }
  }

  return PONode::ptr();
}

void IterativeDeepeningPlanner::solveBreadthFirst()
{
  std::cout << "Breadth First .." << std::endl;

  checked_.insert( root_ );
  queue_.push( root_ );

  for( uint d = 1; d < dmax_; ++d )
  {
    std::cout << "depth = " << d << " queue size:" << queue_.size() << " checked:" << checked_.size() << std::endl;

    breadthFirstExpand( d );

    std::cout << "expanded :" << n_exp_ << std::endl;
  }
}

void IterativeDeepeningPlanner::breadthFirstExpand( uint d )
{
  while( ! queue_.empty() && queue_.front()->depth() <= d )
  {
    auto start = std::chrono::high_resolution_clock::now();


    auto current = queue_.front();
    queue_.pop();

    if( current->isTerminal() )
    {
      std::cout << "terminal for bs:" << current->bs() << std::endl;

      terminals_.push_back( current );

      current->backTrackSolveStatus();

      //std::cout << "root solved:" << root_->isSolved() << std::endl;
    }
    else
    {
      if( ! current->isExpanded() )
      {
        auto start = std::chrono::high_resolution_clock::now();

        n_exp_++;
        current->expand();

        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

        std::cout << "expansion time (ms):" << ms << " n families:" << current->families().size() << std::endl;
      }

      for( auto f : current->families() )
      {
        for( auto c : f )
        {
          if( checked_.count( c ) == 0 )
          {
            checked_.insert( c );
            queue_.push( c );
          }
        }
      }
    }

    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

    //std::cout << "total iteration time (ms):" << ms << std::endl;
  }
}

void IterativeDeepeningPlanner::extractSolutions()
{
  if( ! root_->isSolved() )
  {
    return;
  }
  else
  {
    std::cout << "solution found!" << std::endl;
  }
}

void IterativeDeepeningPlanner::checkIntegrity()
{

}

}
