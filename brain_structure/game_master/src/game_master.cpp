//************************************************
//**********Baxter 2017 Tic-Tac-Toe***************
//*******Nadine Drollinger & Michael Welle********
//************************************************
//*******Game master node - game_master **********
//************************************************

//************************************************
//Description: The Main node that controlls the 
// game. 
//************************************************

#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <game_master/ai_game_masterAction.h> 
#include <game_master/gui_game_masterAction.h>
#include <game_master/grasping_baxter_game_masterAction.h> // links action file 
#include <game_master/camera_game_masterAction.h>
#include <game_master/watch_dog_game_masterAction.h>
#include "game_master/game_manager.h"

using namespace std;

class game_master_boss
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<game_master::gui_game_masterAction> as_gui; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionClient<game_master::ai_game_masterAction> ac_ai;
  actionlib::SimpleActionClient<game_master::grasping_baxter_game_masterAction> ac_grasping_baxter;
  actionlib::SimpleActionClient<game_master::camera_game_masterAction> ac_camera;
  
  std::string action_name_;
  // create messages that are used to published feedback/result
  game_master::gui_game_masterFeedback feedback_gui; // create messages that are used to published feedback
  game_master::gui_game_masterResult result_gui;    // create messages that are used to published result



public:
  
  bool baxter_starts;
  bool gui_start_flag;
  bool stop_game;
  int ai_move;
  int baxter_piece;
  int human_piece;
  int camera_status;
  std::vector<int> gameboard;


  game_master_boss(std::string name) :
    as_gui(nh_, name, boost::bind(&game_master_boss::gui_start_command, this, _1), false),
    action_name_(name), ac_ai("ai_game_master", true), ac_grasping_baxter("grasping_baxter_game_master", true), ac_camera("camera_game_master", true),
    baxter_piece(2),human_piece(1),baxter_starts(true),camera_status(0),gui_start_flag(false),stop_game(false)
  {
    as_gui.start(); //start server that waits for gui

    ROS_INFO("looking for  AI server.");
    ac_ai.waitForServer(); // wait for ai server to be active
    ROS_INFO("found AI server.");

    ROS_INFO("looking for  grasping_baxter server.");
    ac_grasping_baxter.waitForServer(); // wait for ai server to be active
    ROS_INFO("found grasping_baxter server.");

    ROS_INFO("looking for  camera server.");
    ac_camera.waitForServer(); // wait for ai server to be active
    ROS_INFO("found camera server.");    

  }

  ~game_master_boss(void)
  {
  }  

  void print_gameboard(){
    //Print gameboard
    int rows=6;
    int cols=6;
    cout << "*****************" << endl;
    cout << "****Gameboard****" << endl;
    cout << "*****************" << endl;
    for(int i=0;i<rows;i++){
      cout << "**";
      for(int j=0;j<cols;j++){
        cout << "|" << gameboard[j+rows*i];
      }
      cout << "|**" << endl;
      cout << "**_____________**" << endl;    
    }
    cout << "*****************" << endl;
    if(gameboard[36]==baxter_piece)
      cout << "**Next: Baxter***" << endl;
    if(gameboard[36]==human_piece)
      cout << "**Next: Human****" << endl;
    cout << "*****************" << endl;

  }

  bool gameboard_empty(){
    for(int i=0;i<gameboard.size()-1;i++){
      if(gameboard[i]!=0)
        return false;
    }
    return true;
  }
  
  bool player_move_detected(){
    //check board for human pieces (must be same num as baxter pieces)
    int count_baxter_p=0;
    int count_human_p=0;
    for(int i=0;i<gameboard.size()-1;i++){
      if(gameboard[i]==baxter_piece)
        count_baxter_p++;
      if(gameboard[i]==human_piece)
        count_human_p++;
    }
    cout << "move detecter got: baxter pieces: " << count_baxter_p << " human pieces: " << count_human_p << endl; 
    if(baxter_starts){
      if(count_human_p==count_baxter_p)
        return true;
      else
        return false;
    }
    else {
      if(count_human_p==count_baxter_p+1)
        return true;
      else
        return false;
    }
  }


  void request_move_ai(std::vector<int> gameboard)
  {
    //Send the gameboard *******
    game_master::ai_game_masterGoal goal;
    goal.gameboard = gameboard;
    // Need boost::bind to pass in the 'this' pointer
    ac_ai.sendGoal(goal,
                boost::bind(&game_master_boss::received_move_ai, this, _1, _2),
                actionlib::SimpleActionClient<game_master::ai_game_masterAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<game_master::ai_game_masterAction>::SimpleFeedbackCallback());

  }



  void received_move_ai(const actionlib::SimpleClientGoalState& state,
              const game_master::ai_game_masterResultConstPtr& result)
  {
    //ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ai_move =result->best_move;
    //std::cout << "best move is: " << ai_move << std::endl;
  }

  bool wait_for_result_ai(){
    if(ac_ai.waitForResult(ros::Duration(10.0)))
      return true;
    else
      return false;
  }


void request_update_camera(int next_player)
  {
    //ask for camera update
    //ROS_INFO("I want an camera update");
    game_master::camera_game_masterGoal goal;
    goal.next_player = next_player;

    // Need boost::bind to pass in the 'this' pointer
    ac_camera.sendGoal(goal,
                boost::bind(&game_master_boss::received_update_camera, this, _1, _2),
                actionlib::SimpleActionClient<game_master::camera_game_masterAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<game_master::camera_game_masterAction>::SimpleFeedbackCallback());

  }


  void received_update_camera(const actionlib::SimpleClientGoalState& state,
              const game_master::camera_game_masterResultConstPtr& result)
  {
    //ROS_INFO("Finished in state [%s]", state.toString().c_str());
    //ROS_INFO("Answer: %i", result->sequence.back());
    //get new gameboard here!!!
    camera_status=result->status;
    if(camera_status==1)
      gameboard=result->gameboard;
    else
      ROS_WARN("Camera update failed.");   
      
    //TODO: check if only one piece and other stuff
    //ROS_INFO("update_camera_done ");
    //cout << " Status is " << result->fail << endl;    
  }

  bool wait_for_result_camera(){
    if(ac_camera.waitForResult(ros::Duration(10.0)))
      return true;
    else
      return false;
  }



  void request_grasping_baxter(int move)
  {
    //Send which move should be executed
    game_master::grasping_baxter_game_masterGoal goal;
    goal.move = move;

    // Need boost::bind to pass in the 'this' pointer
    ac_grasping_baxter.sendGoal(goal,
                boost::bind(&game_master_boss::received_grasping_done, this, _1, _2),
                actionlib::SimpleActionClient<game_master::grasping_baxter_game_masterAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<game_master::grasping_baxter_game_masterAction>::SimpleFeedbackCallback());

  }

  void received_grasping_done(const actionlib::SimpleClientGoalState& state,
              const game_master::grasping_baxter_game_masterResultConstPtr& result)
  {
    //ROS_INFO("Finished in state [%s]", state.toString().c_str());
    //ROS_INFO("Answer: %i", result->sequence.back());
    //ROS_INFO("got move: ");
  }

  bool wait_for_result_grasping(){
    if(ac_grasping_baxter.waitForResult(ros::Duration(30.0)))
      return true;
    else
      return false;
  }
  //check if the move done by baxeter is performet correctly
  bool baxter_move_correct(){
    //TODO: check entire gameboard
    if(gameboard[ai_move]==baxter_piece)
      return true;
    else
      return false;
  }



  void gui_start_command(const game_master::gui_game_masterGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_gui.progress=20; // progress in %

    // publish info to the console for the user
    ROS_INFO("%s: start received", action_name_.c_str());
	//ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);	
    
    //fedback that everything is ok
    as_gui.publishFeedback(feedback_gui);

    //check if start or stop
    if(goal->start_game==1){
      stop_game=false;
      gui_start_flag=true;
    }
      
    if(goal->start_game==2){
      gui_start_flag=false;
      stop_game=true;
    }
      
    
  
  ROS_INFO("First player: %i", goal->first_player);
    //check first player
    if(goal->first_player ==2)
      baxter_starts=true;
    if(goal->first_player==1)
      baxter_starts=false;
    

    if(success)
    {
      result_gui.game_started = 1;
      ROS_INFO("%s: Done", action_name_.c_str());
      // set the action state to succeeded
      as_gui.setSucceeded(result_gui);
      gui_start_flag=true;
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "gui_game_master");
  ROS_INFO("Start Programm");
  //start the boss
  game_master_boss gmb("gui_game_master");


  //mega main loop
  while(ros::ok()){
    start_place:
    int next_player=0;
    gmb.gui_start_flag=false;
    //wait for start from GUI 
    while(ros::ok() && !gmb.gui_start_flag){
      ROS_INFO_THROTTLE(5, "Waiting for GUI");
      ros::spinOnce();
      ros::Duration(1.0).sleep();
    }

    //select player and prepare board
    //init gameboard 
    gmb.gameboard.clear();
    for (int i = 0; i < 36; ++i)
    {
      gmb.gameboard.push_back(0);
    }
    //player 2 starts (baxtre)
    if(gmb.baxter_starts)
      gmb.gameboard.push_back(gmb.baxter_piece);
    else
      gmb.gameboard.push_back(gmb.baxter_piece);

    //check if gameboard is empty
    if(gmb.gameboard[36]==1)
      next_player=1;
    if(gmb.gameboard[36]==2)
      next_player=2;
    bool temp_flag=true;
    while(ros::ok() && !gmb.gameboard_empty() || temp_flag) {
      temp_flag=false;
      ROS_INFO_THROTTLE(5, "Checking if gameboard is empty");
      ros::spinOnce();
      //performe camera update
      gmb.request_update_camera(next_player);
      if(gmb.wait_for_result_camera()){
        ROS_INFO("Camera update performed with status: %i",gmb.camera_status); 
      }      
      else{
        ROS_ERROR("Failed to performe camera update n time");
      }
      if(!gmb.gameboard_empty())
        break;

      ros::Duration(1.0).sleep();
    }
    if(!gmb.gameboard_empty()){
      ROS_WARN("No empty Gamboard!! please clean up and try again.");
      goto start_place;
    }

    //The game can finnaly start
    //print the gameboard
    gmb.print_gameboard();

    int EOG=false;
    //MAIN game LOOP+++++++++++++++++
    bool baxter_s=gmb.baxter_starts;
    
    while(ros::ok() && !EOG)
    {
      //check for stop
      if(gmb.stop_game)
        goto start_place;
      if(baxter_s){
        baxter_s=true;
        // ask a move from the AI (send the gameboard)
        gmb.request_move_ai(gmb.gameboard);

        //wait for the AI to provide the move
        if(gmb.wait_for_result_ai()){
          ROS_INFO("AI move: %i", gmb.ai_move);
        }    
        else{
          ROS_ERROR("NO AI move in time.");
        }

        // send the move to the baxter_grasping
        gmb.request_grasping_baxter(gmb.ai_move);
        
        //wait for the grasping node to performe the pick place taask
        if(gmb.wait_for_result_grasping()){
          ROS_INFO("Pick place performet");
        }    
        else{
          ROS_ERROR("Failed to pick place in time.");
        }

        //we check if baxter succesfullie placed the piece
        if(gmb.gameboard[36]==1)
          next_player=1;
        if(gmb.gameboard[36]==2)
          next_player=2;
        int baxter_needs_help_counter=0;
        while(ros::ok() && !gmb.baxter_move_correct()){
          //check for stop
          if(gmb.stop_game)
            goto start_place;
          ROS_INFO_THROTTLE(5, "Waiting for baxter confirmation ");
          ros::spinOnce();
          //performe camera update
          gmb.request_update_camera(next_player);
          if(gmb.wait_for_result_camera()){
            ROS_INFO("Camera update performed with status: %i",gmb.camera_status); 
          }      
          else{
            ROS_ERROR("Failed to performe camera update n time");
          }

          ros::Duration(1.0).sleep();
          baxter_needs_help_counter++;
          if(baxter_needs_help_counter>10)
            ROS_INFO("It seems I missed it ... can you move my red piece to field num: %i",gmb.ai_move);
        }
        if(baxter_needs_help_counter>10)
          ROS_INFO("Thanks !!!");

        //print the gameboard
        gmb.print_gameboard();

        //check for EOG
        if(isEOG(gmb.gameboard)){
          ROS_INFO("THE GAME IS OVER");
          if(playerXwin(gmb.gameboard,2)){
            ROS_INFO("BAXTER WINS!!! ");
          }
          if(playerXwin(gmb.gameboard,1)){
            ROS_INFO("HUMAN WINS??? WTF!!!! ");
          }
          if(isdraw(gmb.gameboard)){
            ROS_INFO("draw");
          }
          EOG=true;
          break;
        }

      }
      //make sure baxter can play next time
      baxter_s=true;

      //check for stop
      if(gmb.stop_game)
        goto start_place;
      //Now we wait for the player to make a move
      if(gmb.gameboard[36]==1)
        next_player=1;
      if(gmb.gameboard[36]==2)
        next_player=2;
      int human_needs_help_counter=0;
      while(ros::ok() && !gmb.player_move_detected()){
        //check for stop
        if(gmb.stop_game)
        goto start_place;
        ROS_INFO_THROTTLE(5, "Waiting for  human move");
        ros::spinOnce();
        gmb.request_update_camera(next_player);
        if(gmb.wait_for_result_camera()){
          ROS_INFO("Camera update performed with status: %i",gmb.camera_status); 
        }      
        else{
          ROS_ERROR("Failed to performe camera update n time");
        }
        ros::Duration(1.0).sleep();
        human_needs_help_counter++;

        if(human_needs_help_counter>30)
          ROS_INFO("What are you wating for Meatbag!!!");

      }
      if(human_needs_help_counter>30)
          ROS_INFO("Finnaly!!"); 

     
      //print the gameboard
      gmb.print_gameboard();  

      //check for EOG and stuff
      if(game_manager::isEOG(gmb.gameboard)){
        ROS_INFO("THE GAME IS OVER");
        if(playerXwin(gmb.gameboard,2)){
          ROS_INFO("BAXTER WINS!!! ");
        }
        if(playerXwin(gmb.gameboard,1)){
          ROS_INFO("HUMAN WINS??? WTF!!!! ");
        }
        if(isdraw(gmb.gameboard)){
          ROS_INFO("draw");
        }
        EOG=true;
        break;

      }
      ros::spinOnce();

    }

    ros::spinOnce();
  }


  return 0;
}


