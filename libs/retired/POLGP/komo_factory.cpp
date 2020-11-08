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

#include "komo_factory.h"

//==============KOMOFactory==============================================

void KOMOFactory::registerTask( const rai::String & type, const SymbolGrounder & grounder )
{
  tasks_[ type ] = grounder;
}

std::shared_ptr< ExtensibleKOMO > KOMOFactory::createKomo() const
{
  auto komo = std::make_shared< ExtensibleKOMO >();
  for ( auto task : tasks_ )
  {
    komo->registerTask( task.first, task.second );
  }

  return komo;
}


//==============ExtensibleKOMO==============================================

ExtensibleKOMO::ExtensibleKOMO()
  : KOMO()
{

}

void ExtensibleKOMO::registerTask( const rai::String & type, const SymbolGrounder & grounder )
{
  tasks_[ type ] = grounder;
}

void ExtensibleKOMO::groundTasks( double phase, const Graph& facts, int verbose )
{
  for( Node *n:facts )
  {
    if( ! n->parents.N ) continue; // skip not relevant node

    if( n->keys.N && tasks_.count( n->keys.last() ) != 0 )
    {
      rai::String type = n->keys.last();
      tasks_[ type ]( phase, facts, n, this, verbose ); // ground the symbol
    }
    else if( n->keys.N && n->keys.last().startsWith("komo") )
    {
      HALT("UNKNOWN komo TAG: '" <<n->keys.last() <<"'");
    }
  }
}
