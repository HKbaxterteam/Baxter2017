//************************************************
//**********Baxter 2017 Tic-Tac-Toe***************
//*******Nadine Drollinger & Michael Welle********
//************************************************
//*******AI node - ai ****************************
//************************************************

//************************************************
//Description: getting the gameboard and using 
// min-max search with alpha-beta pruning.
// To make it more interesting, baxter chooses 
// randomly from the possible move-set if there are multiple 
// best moves.
//************************************************

//ros
#include <ros/ros.h>
//action
#include <actionlib/server/simple_action_server.h>
#include <ai/ai_game_masterAction.h>
//c++
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <vector>
//shared header for game function
#include <game_master/game_manager.h>


using namespace std;
//ai class
class ai_boss
{
protected:
  //node handler
  ros::NodeHandle nh_;
  //config action server
  actionlib::SimpleActionServer<ai::ai_game_masterAction> as_ai; 
  std::string action_name_;
  ai::ai_game_masterFeedback feedback_ai; 
  ai::ai_game_masterResult result_ai;    
  

public:
  //vars
  int maxdepth;
  std::vector<int> scoresafe;

  //constructor
  ai_boss(std::string name) :
  as_ai(nh_, name, boost::bind(&ai_boss::ai_start_command, this, _1), false),
  action_name_(name), maxdepth(4)
  {
    ROS_DEBUG_NAMED("ai_node", "Initilizing ai boss");
    //start the ai server
    as_ai.start();
    
  }
  
  //deconstructor
  ~ai_boss(void)
  {
  }

  //function that gets called through the action goal
  void ai_start_command(const ai::ai_game_masterGoalConstPtr &goal)
  {
    // publish info to the console for the user
    ROS_DEBUG_NAMED("ai_node", "%s: start received", action_name_.c_str());
    ROS_DEBUG_NAMED("ai_node", "Looking for the best move");
    //extract gameboard
    vector<int> gameboard = goal->gameboard;
    //helpvars
    vector<int> posMoves;
    std::vector<int> bestmove;
    scoresafe.clear();
    //alpha-beta search
    int score = alphabeta(gameboard,maxdepth,std::numeric_limits<int>::min(),numeric_limits<int>::max(), true);
    //get the next possible boards
    vector<vector<int> > nextgameboards =findposMoves(gameboard);
    //find the max score
    int max=scoresafe[0];
    for(int i =0; i<scoresafe.size();i++){
      ROS_DEBUG_NAMED("ai_node", "Score for board: %i is %i",i,scoresafe[i]);
      if(scoresafe[i]>max){
        max=scoresafe[i];
      }
    }
    //how many best moves?
    int countmax=0;
    for(int i=0;i<scoresafe.size();i++){
      if(scoresafe[i]==max){
        bestmove.push_back(i);
        countmax++;
      }        
    }
    //seed rand
    srand (time(NULL));
    //chose a random move from the bestmove set
    int choosenmove= bestmove[rand() % bestmove.size()];
    ROS_DEBUG_NAMED("ai_node","Found %i best moves.",countmax);

    //find field number for move
    int field;
    for(int i =0;i<gameboard.size()-1;i++){
      if(gameboard[i]!=nextgameboards[choosenmove][i])
        field=i;
    }
    ROS_DEBUG_NAMED("ai_node","Place piece on field: %i",field);
    //send the best move as result back
    result_ai.best_move = field;	// retuŕn best move between 0...48
    ROS_DEBUG_NAMED("ai_node", "%s: Done", action_name_.c_str());
    // set the action state to succeeded
    as_ai.setSucceeded(result_ai);
  }

  //function to find next gameboards
  vector<vector<int> > findposMoves(vector<int> gameboard)
  { 
    //helpervar
    int i=0;
    vector<vector<int> > nextgameboards;
    vector<int> gameboardtemp;
    //loop through all fields
    for(int i =0; i<gameboard.size()-1;i++){
      gameboardtemp=gameboard;
      //if empty switch players and place piece
      if(gameboardtemp[i]==0){
        if(gameboardtemp[36]==1){
          gameboardtemp[36]=2;
          gameboardtemp[i]=1;
        }
        else{
          gameboardtemp[36]=1;
          gameboardtemp[i]=2;
        }
        nextgameboards.push_back(gameboardtemp);
      }

    }
    return nextgameboards;
  } 

  //heuristics
  int ScoringFunction(vector<int> gameboard)
  {
    int score=0;
    int red;
    int blue;
    //win scors
    if(playerXwin(gameboard, 1))
      return -10000000;
    if(playerXwin(gameboard, 2))
      return 10000000;
    if(isdraw(gameboard))
      return 0;
    //eval not EOG board
    for(int i = 0; i < 32; ++i){
      red = 0, blue = 0;
      for(int j = 0; j < 5; ++j){
        if(gameboard[winning_moves[i][j]]== 2){
          red++;
        } 
        else if (gameboard[winning_moves[i][j]]==1){
          blue++;
        }
      }
      if (red > 0 && blue > 0)
        continue;
      if (red == 0 && blue > 0)
        score -= scoremap[blue];
      if (red > 0 && blue == 0)
        score += scoremap[red];
    }
    return score;
  }

  //alphabeta search
  int alphabeta(vector<int> gameboard,int depth,int alpha, int beta, bool maximizingPlayer)
  {
    if (depth == 0 || isEOG(gameboard)) return ScoringFunction(gameboard);
    int bestValue;
    int v;
    //get children
    vector<vector<int> > nextgameboards;
    nextgameboards=findposMoves(gameboard);
    if(maximizingPlayer){
      bestValue=std::numeric_limits<int>::min(); // set -inf
          for (int i = 0; i < nextgameboards.size();i++)
      {
          v=alphabeta(nextgameboards[i],depth-1,alpha,beta,false);
          //save the best move to play it later
          if(depth==maxdepth){
            scoresafe.push_back(std::numeric_limits<int>::min());
            scoresafe[i]=v;
          }
          //update bestval
          bestValue=std::max(bestValue,v);
          //update alpha
          alpha=std::max(alpha,bestValue);
          //beta cut-off
          if(beta<alpha)
            break;
      }
      return bestValue;
    }
    else //minplayer
    {
      bestValue=std::numeric_limits<int>::max();
      for (unsigned i = 0; i < nextgameboards.size();i++)
      {
          v=alphabeta(nextgameboards[i],depth-1,alpha,beta,true);
          //update bestval
          bestValue=std::min(bestValue,v);
          //update beta
          beta=std::min(beta,bestValue);
          //alpha cut off
          if(beta<alpha)
            break;
      }
      return bestValue;
    }
  }


};


//main
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ai_game_master");
  ROS_INFO("Start AI node");
  //start action server
  ai_boss ab("ai_game_master");

  while(ros::ok()){
  ros::spinOnce();
  ros::Duration(0.2).sleep();
 }
   

  return 0;
}
