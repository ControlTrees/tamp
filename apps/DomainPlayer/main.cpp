#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

//===========================================================================

void TEST(FOL_World){
  FOL_World world("boxes_new.kvg");

  auto actions = world.get_actions();
  for(auto& a:actions){ cout <<"DECISION: " <<*a <<endl; }

  for(uint k=0;k<10;k++){
    auto res=world.transition_randomly();
    cout <<"RND TRANSITION: obs=" <<*res.observation <<" r=" <<res.reward <<endl;
  }

  world.get_actions();

  world.make_current_state_new_start();

  world.reset_state();
  world.get_actions();

}

//===========================================================================

void TEST(PlayFOL_World){
  const char *file = "../lgp-learn-self-driving/LGP-merging-1w.g";
  if(rai::argc>1) file = rai::argv[1];

  FOL_World world(file);
  world.verbose = rai::getParameter<int>("verbose", 2);

  for(bool go=true;go;){
    bool terminal = world.is_terminal_state();
    auto actions = world.get_actions();
    cout <<"********************" <<endl;
    cout <<"STATE: ";
    world.get_info(MCTS_Environment::writeState);
    cout <<"\nCHOICES:" <<endl;
    cout <<"(q) quit" <<endl;
    cout <<"(r) reset_state" <<endl;
    cout <<"(m) make_current_initial" <<endl;
    uint c=0;
    if(!terminal) for(auto& a:actions){ cout <<"(" <<c++ <<") DECISION: " <<*a <<endl; }

    char cmd='1';
    std::cin >>cmd;
    cout <<"COMMAND: '" <<cmd <<"'" <<endl;

    if(!terminal && cmd>='0' && cmd<='9'){
      auto &a = actions[int(cmd-'0')];
      cout <<"executing decision " <<*a <<endl;
      auto res=world.transition(a);
      cout <<"->  result: obs=" <<*res.observation <<" reward=" <<res.reward <<endl;
    }else switch(cmd){
      case 'q': go=false; break;
      case 'r': world.reset_state(); break;
      case 'm': world.make_current_state_new_start(); break;
      default: LOG(-1) <<"command '" <<c <<"' not known";
    }
  }
}

//===========================================================================

int main(int argn, char** argv){
  rai::initCmdLine(argn, argv);
  rnd.clockSeed();

//  testMCTS();

  testPlayFOL_World();
}
