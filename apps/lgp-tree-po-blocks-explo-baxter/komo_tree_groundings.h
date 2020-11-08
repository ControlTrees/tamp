#pragma once

#include <graph_planner.h>
#include <komo_factory.h>

void groundTreeInit( const mp::TreeBuilder& tb, KOMO_ext* komo, int verbose );
void groundTreePickUp( const mp::Interval& interval, const mp::TreeBuilder& tb, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose );
void groundTreePutDown( const mp::Interval& interval, const mp::TreeBuilder& tb, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose );
void groundTreeCheck( const mp::Interval& interval, const mp::TreeBuilder& tb, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose );
