#include <logic_engine.h>

namespace matp
{

// LogicEngine
LogicEngine::LogicEngine( const LogicEngine & engine ) // copy ctor
{
  if( ! engine.descriptionFile_.empty() )
  {
    init( engine.descriptionFile_ );
  }
}

LogicEngine& LogicEngine::operator=( const LogicEngine & engine ) // assignment operator
{
  if( ! engine.descriptionFile_.empty() )
  {
    init( engine.descriptionFile_ );
  }
  return *this;
}

void LogicEngine::init( const std::string & file )
{
  descriptionFile_ = file;
  engine_ = std::shared_ptr< FOL_World >( new FOL_World( descriptionFile_.c_str() ) );

  parseNumberOfAgents();
  parseActions();

  engine_->reset_state();
}

uint LogicEngine::totalActionsNumber( uint agentId ) const
{
  if( agentId < actionsNames_.size() )
  {
    return actionsNames_[ 0 ].size();
  }
  else
  {
    return 0;
  }
}

std::vector<std::string> LogicEngine::getPossibleActions( uint agentId )
{
  if( ! initialized() ) throw NotInitialized();

  std::vector<std::string> filteredActions;

  const auto& actions = engine_->get_actions();

  for( const auto& action : actions )
  {
    if( isOfAgent( action, agentId ) || agentNumber_ == 1 )
    {
      filteredActions.push_back( actionToString( action ) );
    }
  }

  // test
  if( engine_->is_terminal_state() )
  {
    CHECK_EQ( actions.size(), 0, "terminal state still has possible actions!" );
  }
  //

  return filteredActions;
}

void LogicEngine::transition( const std::string & action )
{
  auto actions = engine_->get_actions();

  for( const auto& a : actions )
  {
    if( actionToString( a ) == action )
    {
      engine_->transition( a );
    }
  }
}

void LogicEngine::setState( const std::string & state )
{
  auto mlrState = rai::String( state );
  engine_->reset_state();
  engine_->set_state( mlrState );
}

std::string LogicEngine::getState() const
{
  return getStateStr( *engine_ );
}

void LogicEngine::parseNumberOfAgents()
{
  Graph KB;
  KB.read( FILE( descriptionFile_.c_str() ) );

  uint agentIDCandidate = 1;
  std::string agentNameCandidate = agentPrefix_ + std::to_string( agentIDCandidate ) + agentSuffix_;

  while( KB[ agentNameCandidate.c_str() ] != nullptr )
  {
    agentIDCandidate++;
    agentNameCandidate = agentPrefix_ + std::to_string( agentIDCandidate ) + agentSuffix_;
  }

  agentNumber_ = agentIDCandidate;
}

void LogicEngine::parseActions()
{
  const auto& actions = engine_->decisionRules;

  actionsNames_ = std::vector< std::vector< std::string > > ( agentNumber_ );

  for( const auto& actionNode : actions )
  {
    const auto& actionName = actionToString( actionNode );

    auto agentId = getAgentId( actionName );

    if( agentId == -1 )
    {
      if( agentNumber_ == 1 )
      {
        agentId = 0;
      }
      else
      {
        throw IncoherentDefinition();
      }
    }

    actionsNames_[ agentId ].push_back( actionName );
  }
}

}
