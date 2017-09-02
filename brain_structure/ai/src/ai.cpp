
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ai/ai_game_masterAction.h>

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include <game_master/game_manager.h>

#include <vector>

using namespace std;
class ai_boss
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<ai::ai_game_masterAction> as_ai; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  ai::ai_game_masterFeedback feedback_ai; // create messages that are used to published feedback
  ai::ai_game_masterResult result_ai;    // create messages that are used to published result

  int maxdepth;

  //std::vector<int> scoremap;
  //std::vector<std::vector<int> > winning_moves;
  std::vector<int> scoresafe;
  //const int winning_moves[32][5];

public:

	bool ai_start_flag;

  ai_boss(std::string name) :
  as_ai(nh_, name, boost::bind(&ai_boss::ai_start_command, this, _1), false),
  action_name_(name), ai_start_flag(false),maxdepth(4)
  {
    as_ai.start();
    
  }

  ~ai_boss(void)
  {
  }


  void ai_start_command(const ai::ai_game_masterGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;
    vector<int> gameboard = goal->gameboard;
    vector<int> posMoves;
    std::vector<int> bestmove;

    ROS_INFO("calculating move");
    cout << gameboard[0] << endl;
    cout << gameboard.size()<< endl;

    for(int i = 0;i<gameboard.size();i++){
      cout << gameboard[i] ;
    }
    ROS_INFO("calculating movesss");
    //CALCULATE THE AI MOVE******************************
    //***************************************************
    
    //give me best move
    scoresafe.clear();

    int score = alphabeta(gameboard,maxdepth,std::numeric_limits<int>::min(),numeric_limits<int>::max(), true);
    
    vector<vector<int> > nextgameboards =findposMoves(gameboard);

    //find best move
    int max=scoresafe[0];
    for(int i =0; i<scoresafe.size();i++){
      cout << "score for: " << i << " is " << scoresafe[i] << endl;

      if(scoresafe[i]>max){
        max=scoresafe[i];
      }
    }
    int countmax=0;
    for(int i=0;i<scoresafe.size();i++){
      if(scoresafe[i]==max){
        bestmove.push_back(i);
        countmax++;
      }
        
    }
    srand (time(NULL));
    int choosenmove= bestmove[rand() % bestmove.size()];
    cout << "we have: " << countmax << " best move " << endl;

    cout << "choosenmove " << choosenmove << endl;
  //find real number
  int field;
  for(int i =0;i<gameboard.size()-1;i++){
    if(gameboard[i]!=nextgameboards[choosenmove][i])
      field=i;
  }
  cout << "place piece on field " << field << endl;


    feedback_ai.progress=100;	// progess in percentage

    // publish info to the console for the user
    ROS_INFO("%s: start received", action_name_.c_str());
	  
    //fedback that everything is ok
    as_ai.publishFeedback(feedback_ai);
    

    if(success)
    {
    	ROS_INFO("calculating move done");
      result_ai.best_move = field;	// retuŕn best move between 0...48
      ROS_INFO("%s: Done", action_name_.c_str());
      // set the action state to succeeded
      as_ai.setSucceeded(result_ai);
      ai_start_flag=true;
    }
  }

  vector<vector<int> > findposMoves(vector<int> gameboard)
  { int i=0;
    vector<vector<int> > nextgameboards;
    vector<int> gameboardtemp;
    for(int i =0; i<gameboard.size()-1;i++){
      gameboardtemp=gameboard;
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


  int ScoringFunction(vector<int> gameboard)
  {
    int score=0;
    int red;
    int blue;
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
            cout << scoresafe.size() << endl;
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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ai_game_master");
  ROS_INFO("Start AI node");
  //start action server
  ai_boss ab("ai_game_master");

  while(ros::ok()){
  ros::spinOnce();
  //ROS_INFO("AIIIIIIIIIII Still alive");
  ros::Duration(1.0).sleep();
 }
   

  return 0;
}
