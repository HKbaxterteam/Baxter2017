
#include <iostream>
#include <vector>
#include <cstdlib>
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;

//proto
bool isdraw(vector<int> gameboard);
bool isEOG(vector<int> gameboard);
int ScoringFunction(vector<int> gameboard);
bool playerXwin(vector<int> gameboard, int player);
int alphabeta(vector<int> gameboard,int depth,int alpha, int beta, bool maximizingPlayer);
vector<vector<int> > findposMoves(vector<int> gameboard);
int maxdepth=3;
int bestmove=0;
const int scoremap[6] = {
        0, 1, 10, 1000, 100000, 1000000 };
//gameboard
// | 0, 1


const int winning_moves[32][5] = 
	{
        // 12 horizotal winning
        {0, 1,  2,  3, 4},
        {1, 2,  3,  4,  5},
        {6, 7,  8,  9,  10},
        {7, 8,  9,  10, 11},        
        {12,  13,  14,  15,  16},
        {13,  14,  15,  16,  17},
        {18,  19,  20,  21,  22},
        {19,  20,  21,  22,  23},
        {24,  25,  26,  27,  28},
        {25,  26,  27,  28,  29},        
        {30,  31,  32,  33,  34},
        {31,  32,  33,  34,  35},

        // 12 vertical winning
        {0,  6,   12,  18,  24},
        {6,  12,  18,  24,  30},
        {1,  7,   13,  19,  25},
        {7,  13,  19,  25,  31},        
        {2,  8,   14,  20,  26},
        {8,  14,  20,  26,  32},
        {3,  9,   15,  21,  27},
        {9,  15,  21,  27,  33},
        {4,  10,  16,  22,  28},
        {10, 16,  22,  28,  34},        
        {5,  11,  17,  23,  29},
        {11, 17,  23,  29,  35},


        // 4 maindiagonal winning

        {0,  7,   14,  21,  28},
        {7,  14,  21,  28,  35},        
        {5,  10,  15,  20,  25},
        {10, 15,  20,  25,  30},

        // 4 other diagonal 


        {1,  8,   15,  22,  29},
        {6,  13,  20,  27,  34},
        {4,  9,   14,  19,  24},
        {11, 16,  21,  26,  31}
 	};

int main(int argc, char **argv)
{

 ros::init(argc, argv, "talker");
 ros::NodeHandle n;
 ros::Rate loop_rate(10);
 vector<int> gameboard;
 vector<int> posMoves;

  int a=0;
  int i=0;
  int player=1;

  while (i<=35) // create dummy board
  {
    gameboard.push_back(a);
    cout << gameboard[i] << endl;
    i++;
  }
  gameboard.push_back(2); // player 1 starts
  //gameboard.push_back(36)=player;

  //nice boards
  gameboard[0]=1;
  gameboard[1]=1;
  gameboard[2]=1;
  gameboard[6]=2;
  gameboard[7]=2;
  gameboard[8]=2;
	cout << "gameboard stores " << int(gameboard.size()) << " numbers.\n";

	vector<vector<int> > nextgameboards =findposMoves(gameboard);

	//print possible boards
	for(int i=0;i<nextgameboards.size();i++){
		for(int j=0;j<gameboard.size()-1;j++){
			cout << nextgameboards[i][j];
		}
		cout << " " << endl;
	}

	//give me best move

	int score = alphabeta(gameboard,maxdepth,std::numeric_limits<int>::min(),numeric_limits<int>::max(), true);

	cout << "bestmove " << bestmove << endl;
	//find real number
	int field;
	for(int i =0;i<gameboard.size()-1;i++){
		if(gameboard[i]!=nextgameboards[bestmove][i])
			field=i;
	}
	cout << "place piece on field " << field << endl;



}



//------- find possible moves

	vector<vector<int> > findposMoves(vector<int> gameboard)
	{	int i=0;
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
			{

				win_count++;
			}

			if(win_count==5)
			{
				return true;
			}
		}
	}
	return false;

}

 bool isdraw(vector<int> gameboard)
 {
 	for(int i=0; i<gameboard.size()-1; i++)
 	{
 		if(gameboard[i]==0)
 			return false;
 	}
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
		{
			return -1000000;
		}
		if(playerXwin(gameboard, 2))
		{
			return 1000000;
		}
		if(isdraw(gameboard))
		{
			return 0;
		}

		//eval not EOG board
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
                continue;
            if (red == 0 && blue > 0)
                score -= scoremap[blue];
            if (red > 0 && blue == 0)
                score += scoremap[red];
        }
        return score;
        
        return 0;















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