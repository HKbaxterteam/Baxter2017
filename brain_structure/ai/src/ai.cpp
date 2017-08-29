
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ai/ai_game_masterAction.h>

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

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
  int bestmove;

  std::vector<int> scoremap;
  std::vector<std::vector<int> > winning_moves;
  //const int winning_moves[32][5];

public:

	bool ai_start_flag;

  ai_boss(std::string name) :
  as_ai(nh_, name, boost::bind(&ai_boss::ai_start_command, this, _1), false),
  action_name_(name), ai_start_flag(false),maxdepth(4),bestmove(0)
  {
    as_ai.start();
    //set up score map
    scoremap.push_back(0);
    scoremap.push_back(1);
    scoremap.push_back(10);
    scoremap.push_back(1000);
    scoremap.push_back(1000000);
    scoremap.push_back(100000000);
    //set up the winning moves
    std::vector<int> temp;
    // 12 horizontal lines
    temp.push_back(0); temp.push_back(1); temp.push_back(2); temp.push_back(3);temp.push_back(4);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(1); temp.push_back(2); temp.push_back(3); temp.push_back(4);temp.push_back(5);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(6); temp.push_back(7); temp.push_back(8); temp.push_back(9);temp.push_back(10);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(7); temp.push_back(8); temp.push_back(9); temp.push_back(10);temp.push_back(11);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(12); temp.push_back(13); temp.push_back(14); temp.push_back(15);temp.push_back(16);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(13); temp.push_back(14); temp.push_back(15); temp.push_back(16);temp.push_back(17);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(18); temp.push_back(19); temp.push_back(20); temp.push_back(21);temp.push_back(22);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(19); temp.push_back(20); temp.push_back(21); temp.push_back(22);temp.push_back(23);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(24); temp.push_back(25); temp.push_back(26); temp.push_back(27);temp.push_back(28);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(25); temp.push_back(26); temp.push_back(27); temp.push_back(28);temp.push_back(29);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(30); temp.push_back(31); temp.push_back(32); temp.push_back(33);temp.push_back(34);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(31); temp.push_back(32); temp.push_back(33); temp.push_back(34);temp.push_back(35);
    winning_moves.push_back(temp);
    temp.clear();
    // 12 vertical winning
    temp.push_back(0); temp.push_back(6); temp.push_back(12); temp.push_back(18);temp.push_back(24);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(6); temp.push_back(12); temp.push_back(18); temp.push_back(24);temp.push_back(30);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(1); temp.push_back(7); temp.push_back(13); temp.push_back(14);temp.push_back(25);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(7); temp.push_back(13); temp.push_back(19); temp.push_back(25);temp.push_back(31);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(2); temp.push_back(8); temp.push_back(14); temp.push_back(15);temp.push_back(26);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(8); temp.push_back(14); temp.push_back(20); temp.push_back(26);temp.push_back(32);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(3); temp.push_back(9); temp.push_back(15); temp.push_back(21);temp.push_back(27);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(9); temp.push_back(15); temp.push_back(21); temp.push_back(27);temp.push_back(33);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(4); temp.push_back(10); temp.push_back(16); temp.push_back(22);temp.push_back(28);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(10); temp.push_back(16); temp.push_back(22); temp.push_back(28);temp.push_back(34);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(5); temp.push_back(11); temp.push_back(17); temp.push_back(23);temp.push_back(29);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(11); temp.push_back(17); temp.push_back(23); temp.push_back(29);temp.push_back(35);
    winning_moves.push_back(temp);
    temp.clear();
    //4 main diagonal
    temp.push_back(0); temp.push_back(7); temp.push_back(14); temp.push_back(21);temp.push_back(28);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(7); temp.push_back(14); temp.push_back(21); temp.push_back(28);temp.push_back(35);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(5); temp.push_back(10); temp.push_back(15); temp.push_back(20);temp.push_back(25);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(10); temp.push_back(15); temp.push_back(20); temp.push_back(25);temp.push_back(30);
    winning_moves.push_back(temp);
    temp.clear();
    //4 other diagonal
    temp.push_back(1); temp.push_back(8); temp.push_back(15); temp.push_back(22);temp.push_back(29);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(6); temp.push_back(13); temp.push_back(20); temp.push_back(27);temp.push_back(34);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(4); temp.push_back(9); temp.push_back(14); temp.push_back(19);temp.push_back(24);
    winning_moves.push_back(temp);
    temp.clear();
    temp.push_back(11); temp.push_back(16); temp.push_back(21); temp.push_back(26);temp.push_back(31);
    winning_moves.push_back(temp);
    temp.clear();

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

    ROS_INFO("calculating move");
    //CALCULATE THE AI MOVE******************************
    //***************************************************
    
    //give me best move

    int score = alphabeta(gameboard,maxdepth,std::numeric_limits<int>::min(),numeric_limits<int>::max(), true);
    
    vector<vector<int> > nextgameboards =findposMoves(gameboard);

    cout << "bestmove " << bestmove << endl;
  //find real number
  int field;
  for(int i =0;i<gameboard.size()-1;i++){
    if(gameboard[i]!=nextgameboards[bestmove][i])
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


  bool playerXwin(vector<int> gameboard, int player){
    
    int win_count=0;
    for(int i=0; i<32; i++)
    { 
      win_count=0;
      for (int j=0; j<5;j++)
      {
        if( gameboard[winning_moves[i][j]]==player)
          win_count++; 
        if(win_count==5) 
        return true;
      }
    }
    return false;
  }

  bool isdraw(vector<int> gameboard)
  {
    int red;
    int blue;
    int count=0;
    for(int i = 0; i < 32; ++i){
      red = 0, blue = 0;
      for(int j = 0; j < 5; ++j){
        if(gameboard[winning_moves[i][j]]== 2){
          red++;
        } else if (gameboard[winning_moves[i][j]]==1){
          blue++;
        }
      }
      if (red > 0 && blue > 0)
                count++;
    }
    if(count<35)
      return false;

    return true;
  }


  bool isEOG(vector<int> gameboard)
  {
    if(isdraw(gameboard) || playerXwin(gameboard,1) || playerXwin(gameboard,2))
      return true;

    return false;
  }


  int ScoringFunction(vector<int> gameboard)
  {
    int score=0;
    int red;
    int blue;
    if(playerXwin(gameboard, 1))
      return -100000000;
    if(playerXwin(gameboard, 2))
      return 100000000;
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
          if(depth==maxdepth && v>bestValue)
                bestmove= i;
              //update bestval
          bestValue=std::max(bestValue,v);
          //update alpha
          alpha=std::max(alpha,bestValue);
          //beta cut-off
          if(beta<=alpha)
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
          if(beta<=alpha)
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
