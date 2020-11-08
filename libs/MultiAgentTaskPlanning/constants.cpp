#include <constants.h>

#include <boost/tokenizer.hpp>
#include <boost/algorithm/string/replace.hpp>

namespace matp
{

std::string actionToString( Node * action )
{
  std::stringstream ss;
  ss << *action;
  return ss.str();
}

std::string actionToString( const FOL_World::Handle & action )
{
  std::stringstream ss;
  ss << *action;
  return ss.str();
}

int getAgentId( const std::string actionName )
{
  //auto prefix = actionName.substr( 0, agentPrefix_.size() );
  auto prefixIndex = actionName.find( agentPrefix_ );

  if( prefixIndex == std::string::npos )
  {
    return -1;
  }

  auto prefix = actionName.substr( prefixIndex, agentPrefix_.size() );

  if( prefix != agentPrefix_ )
  {
    return -1;
  }

  auto actionNameFiltered = actionName.substr( prefixIndex + agentPrefix_.size(), actionName.size() );
  auto suffixStartIndex = actionNameFiltered.find_first_of( agentSuffix_ );
  auto idString = actionNameFiltered.substr( 0, suffixStartIndex );

  return std::stoi( idString );
}

bool isOfAgent( const std::string & str, uint agentId )
{
  auto agentPattern = agentPrefix_ + std::to_string( agentId ) + agentSuffix_;

  return str.find( agentPattern ) != -1;
}

bool isOfAgent( const FOL_World::Handle & action, uint agentId )
{
  auto actionFact = actionToString( action );

  return isOfAgent( actionFact, agentId );
}

StringA nodeToStringA( Node * facts )
{
  StringA factStringA;

  for( auto f : facts->parents )
  {
    factStringA.append( f->keys.last() );
  }

  return factStringA;
}

std::set< std::string > getFilteredFacts( const std::string & state )
{
  //std::string filteredResult("{");
  std::set< std::string > facts;

  using tokenizer = boost::tokenizer<boost::char_separator<char> >;

  boost::char_separator<char> sep( "," );
  tokenizer tokens( state, sep );

  for( auto fact : tokens )
  {
    boost::replace_all(fact, "{", "");
    boost::replace_all(fact, "}", "");

    if( ! fact.empty() && ( fact.find( "decision(") == std::string::npos ) && ( fact.find( "komo") == std::string::npos ) )
    {
      //fact.append( "," );
      facts.insert( fact );
    }
  }

  return facts;
}

std::vector< std::string > sortFacts( const std::vector< std::string > & facts )
{
  std::vector< std::string > sortedFacts = facts;

  std::sort( sortedFacts.begin(), sortedFacts.end() );

  return sortedFacts;
}


//std::string concatenateFacts( const std::set< std::string >  & facts )
//{
//  std::string filteredResult("{");

//  bool first = true;
//  for( auto fact : facts )
//  {
//    if( ! first )
//    {
//      filteredResult.append(",");
//    }
//    filteredResult.append(fact);

//    first = false;
//  }

//  filteredResult.append( "}" );

//  return filteredResult;
//}

bool isObservable( const std::string & fact )
{
  if( fact.find( notObservableTag_ ) != std::string::npos )
  {
    return false;
  }

  return true;
}

std::string getStateStr( FOL_World & fol )
{
//  std::stringstream ss;
//  fol.write_state( ss );
//  return ss.str();

  std::stringstream ss;

  ss << "{";
  for( auto fact : *fol.getState() )
  {
    std::stringstream sss;
    sss << *fact;
    auto factsStr = sss.str();

    if( factsStr.back() != ',' )
    {
      factsStr.push_back( ',' );
    }

    ss << factsStr;
  }
  ss << "}";

  return ss.str();
}

std::string concatenateFacts( const std::set< std::string > & facts )
{
  if( facts.empty() )
  {
    return "";
  }

  std::stringstream ss;
  ss << "{";
  for( auto fact : facts )
  {
    ss << fact << ",";
  }
  ss << "}";
  return ss.str();
}

std::set< std::string > getObservableFacts( const std::set< std::string > & facts )
{
  std::set< std::string > observableFacts;

  for( auto fact : facts )
  {
    if( isObservable( fact ) )
    {
      observableFacts.insert( fact );
    }
  }

  return observableFacts;
}

std::string getObservableState( const std::string & state )
{
  std::set< std::string > facts = getFilteredFacts( state );

  std::set< std::string > observableFacts = getObservableFacts( facts );

  std::string observableState = concatenateFacts( observableFacts );

  return observableState;
}

std::set< std::string > getEmergingFacts( const std::set< std::string > & intersectingFacts, const std::set< std::string > & fullFacts )
{
  std::set< std::string > observations = fullFacts;

  for( auto fact : intersectingFacts )
  {
    observations.erase( fact );
  }

  return observations;
}

}
