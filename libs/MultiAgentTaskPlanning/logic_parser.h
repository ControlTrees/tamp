#pragma once

#include <stdexcept>

#include <queue>
#include <string>
#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <logic_engine.h>

#include <constants.h>

namespace matp
{

class FolFileNotFound: public std::exception
{
  virtual const char* what() const throw()
  {
    return "Fol file not found";
  }
};

class LogicParser
{
public:
  void parse( const std::string & description );

  // getters
  bool successfullyParsed() const { return successfullyParsed_; }
  uint agentNumber() const;
  uint totalActionsNumber( uint agentId ) const;
  std::vector< std::string > possibleStartStates() const { return startStates_; }
  std::vector< double >      egoBeliefState() { return egoBeliefState_; }
  LogicEngine engine() const { return engine_; }

private:
  void buildPossibleStartStates( const std::string & description );
  void parseBeliefState( const std::string & description );
  void checkCoherence();
  void setUpEngine( const std::string & description );

private:
  bool successfullyParsed_ = false;
  LogicEngine engine_;
  uint agentNumber_ = 0;
  std::vector< double > egoBeliefState_;
  std::vector< std::string > startStates_;
};

} // namespace matp
