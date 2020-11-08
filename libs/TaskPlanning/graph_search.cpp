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

#include <graph_search.h>
#include <policy_builder.h>
#include <graph_printer.h>
#include <yens.h>

#include <chrono>
#include <functional>

//=====================free functions======================

namespace tp
{

void GraphSearchPlanner::setFol( const std::string & folDescription )
{
  const rai::String notObservableTag = "NOT_OBSERVABLE";

  Graph KB;
  KB.read( FILE( folDescription.c_str() ) );
  //KB.isDoubleLinked = false;
  // fully observable case
  if( KB[ beliefStateTag_ ] == nullptr )
  {
    // create dummy array
    folEngines_ = rai::Array< std::shared_ptr<FOL_World> > ( 1 );
    std::shared_ptr<FOL_World> fol = std::make_shared<FOL_World>();
    fol->init(FILE(folDescription.c_str()));
    folEngines_( 0 ) = fol;
    fol->reset_state();
    //fol->KB.isDoubleLinked = false;
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
    folEngines_ = rai::Array< std::shared_ptr<FOL_World> > ( nWorlds );
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
      folEngines_(w) = fol;
      bs_(w) = probability;
      folEngines_(w)->reset_state();
    }

    // check that the belief state sums to 1
    double total = 0;
    for( auto p : bs_ ) total += p;

    CHECK( total == 1.00, "wrong belief state definition, the total of the probabilities doesn't sum to 1" );
  }
}

void GraphSearchPlanner::buildGraph()
{
  buildGraphImpl();
}

void GraphSearchPlanner::solve()
{
  std::cout << "GraphSearchPlanner::solveSymbolically" << std::endl;

  if( ! graph_ )
  {
    buildGraphImpl();
  }

  //Yens solver( folEngines_ );
  //auto policies = solver.solve( weightedGraph_, 0 );

  Dijkstra solver( folEngines_ );
  policy_ = solver.solve( weightedGraph_, weightedGraph_->root() );
}

void GraphSearchPlanner::integrate( const Policy::ptr & policy )
{ 
  std::list< PolicyNode::ptr > queue;
  queue.push_back( policy->root() );

  while( ! queue.empty() )
  {
    auto node = queue.front();
    queue.pop_front();

    for( auto c : node->children() )
    {
      if( c->status() == PolicyNode::INFORMED )
      {
        weightedGraph_->setReward( node->id(), c->id(), c->lastReward() );
      }

      queue.push_back( c );
    }
  }
}

Policy::ptr GraphSearchPlanner::getPolicy() const
{
  return policy_;
}

MotionPlanningOrder GraphSearchPlanner::getPlanningOrder() const
{
  MotionPlanningOrder po( getPolicy()->id() );

  //
  po.setParam( "type", "markovJointPath" );
  //

  return po;
}

void GraphSearchPlanner::saveGraphToFile( const std::string & filename )
{
  if( ! graph_ )
  {
    return;
  }

  std::ofstream file;
  file.open( filename );

  GraphPrinter printer( file );
  printer.print( graph_->root(), graph_->terminals() );

  file.close();
}

void GraphSearchPlanner::buildGraphImpl()
{
  POGraphNode::ptr root = std::make_shared< POGraphNode >( folEngines_, bs_ );

  std::queue< POGraphNode::ptr > queue;
  std::list < POGraphNode::ptr > terminals;

  queue.push( root );

  while( ! queue.empty() )
  {
    auto current = queue.front();
    queue.pop();

    if( current->isTerminal() )
    {
      std::cout << "terminal for bs:" << current->bs() << std::endl;

      terminals.push_back( current );
    }
    else
    {
      if( ! current->isExpanded() )
      {
        auto newNodes = current->expand();

        for( auto n : newNodes )
        {
          queue.push( n );
        }

        if( queue.size() % 50 == 0 )
        {
          std::cout << "queue_.size():" << queue.size() << std::endl;
        }
      }
    }
  }

  std::cout << "Graph build:" << root->shared_node_list()->size() << " number of terminal nodes:" << terminals.size() << std::endl;

  graph_         = std::make_shared< POGraph >( root, terminals );
  weightedGraph_ = std::make_shared< POWeightedGraph >( graph_, initialReward_ );
}

void GraphSearchPlanner::yen( uint k )   // generates a set of policies
{

}

void GraphSearchPlanner::checkIntegrity()
{

}

}
