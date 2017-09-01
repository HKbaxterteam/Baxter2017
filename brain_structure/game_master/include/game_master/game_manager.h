#ifndef game_manager
#define game_manager

using namespace std;

const int scoremap[6] = {
        0, 1, 10, 1000, 100000, 1000000 };

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
    if(count<32)
      return false;

    return true;
 }


 bool isEOG(vector<int> gameboard)
 {

  if(isdraw(gameboard) || playerXwin(gameboard,1) || playerXwin(gameboard,2))
    return true;

  return false;
 }

#endif