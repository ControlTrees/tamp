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


#pragma once

#include <memory>

#include <KOMO/komo.h>

//=====ExtensibleKOMO==============================================

class ExtensibleKOMO : public KOMO
{
  typedef std::function<void( double time, const Graph& facts, Node *n, KOMO *, int verbose )> SymbolGrounder;

public:
  typedef std::shared_ptr< ExtensibleKOMO > ptr;
public:
  ExtensibleKOMO();

  void registerTask( const rai::String & type, const SymbolGrounder & grounder );
  void groundTasks( double phase, const Graph& facts, int verbose=0 );

private:
  std::map< rai::String, SymbolGrounder > tasks_;
};

//=====ExtensibleKOMO==============================================

class KOMOFactory
{
  typedef std::function<void( double, const Graph& facts, Node *n, KOMO *, int verbose )> SymbolGrounder;

public:
  void registerTask( const rai::String & type, const SymbolGrounder & grounder );
  std::shared_ptr< ExtensibleKOMO > createKomo() const;
private:
  std::map< rai::String, SymbolGrounder > tasks_;
};
struct PartiallyObservableNode;
