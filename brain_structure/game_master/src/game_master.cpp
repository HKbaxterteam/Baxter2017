//************************************************
//**********Baxter 2017 Tic-Tac-Toe***************
//*******Nadine Drollinger & Michael Welle********
//************************************************
//*******Game master node - game_master **********
//************************************************

//************************************************
// Description: The main node that controlls the 
// game. 
//************************************************

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <game_master/ai_game_masterAction.h> 
#include <game_master/gui_game_masterAction.h>
#include <game_master/grasping_baxter_game_masterAction.h> // links action file 
#include <game_master/camera_game_masterAction.h>
#include <game_master/game_master_guiAction.h>
#include "game_master/game_manager.h"
#include <std_msgs/String.h>
//opencv for faces
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/video/background_segm.hpp"
#include "opencv2/video/tracking.hpp"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include <string> 

using namespace cv;
using namespace std;

class game_master_boss
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<game_master::gui_game_masterAction> as_gui; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionClient<game_master::ai_game_masterAction> ac_ai;
  actionlib::SimpleActionClient<game_master::grasping_baxter_game_masterAction> ac_grasping_baxter;
  actionlib::SimpleActionClient<game_master::camera_game_masterAction> ac_camera;
  actionlib::SimpleActionClient<game_master::game_master_guiAction> ac_guistatus;
  image_transport::ImageTransport it_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  game_master::gui_game_masterFeedback feedback_gui; // create messages that are used to published feedback
  game_master::gui_game_masterResult result_gui;    // create messages that are used to published result
  image_transport::Publisher image_pub_face_;


public:
  
  bool baxter_starts;
  bool gui_start_flag;
  bool gui_ground_state_flag;
  bool gui_check_reach_flag;
  bool stop_game;
  bool feedback_grasping;
  int ai_move;
  int baxter_piece;
  int human_piece;
  int camera_status;
  int game_pieces_baxter;
  std::vector<int> gameboard;

 // define action server=as and action client =ac
  game_master_boss(std::string name) :
    as_gui(nh_, name, boost::bind(&game_master_boss::gui_start_command, this, _1), false),
    action_name_(name),it_(nh_), ac_ai("ai_game_master", true), ac_grasping_baxter("grasping_baxter_game_master", true),
    ac_camera("camera_game_master", true), ac_guistatus("game_master_gui", true), baxter_piece(2),human_piece(1),baxter_starts(true),
    camera_status(0),gui_start_flag(false),stop_game(false),game_pieces_baxter(18),feedback_grasping(false),
    gui_ground_state_flag(false), gui_check_reach_flag(false)
  {
    as_gui.start(); //start server that waits for gui

    ROS_DEBUG_NAMED ("game_master","Waiting for  AI server." );
    ac_ai.waitForServer(); // wait for ai server to be active
    ROS_DEBUG_NAMED ("game_master","AI server activated." );

    
	ROS_DEBUG_NAMED ("game_master","Waiting for grasping baxter server." );
    ac_grasping_baxter.waitForServer(); // wait for ai server to be active
    ROS_DEBUG_NAMED ("game_master","Found grasping baxter server." );

    ROS_DEBUG_NAMED ("game_master","Waiting for Camera." );
    ac_camera.waitForServer(); // wait for ai server to be active
    ROS_DEBUG_NAMED ("game_master","Found for Camera." );

    //action communication
  //ROS_INFO("Looking for gui server.");
  ROS_DEBUG_NAMED ("game_master","Waitung for GUI server" );
  ac_guistatus.waitForServer();
  //ROS_INFO("found gui server");  
  ROS_DEBUG_NAMED ("game_master","found GUI server." );
  //face puplisher (latch = true)
  image_pub_face_ = it_.advertise("/robot/xdisplay", 1,true);
    
  }
  //deconstructor
  ~game_master_boss(void)
  {
  }  

  void eog_reaction(int result){
    // win draw or lose?
    switch(result){
      //Baxter win
      case 1: send_gui_status("Baxter WINS","MUAHHAHAHHAHHAHAHA YIPPPIIIEEE "); 
              for(int i=0;i<20;i++){
                show_face(12);
                ros::Duration(0.25).sleep();
                show_face(13);
                ros::Duration(0.25).sleep();
              }
              break;
      //Draw
      case 2: send_gui_status("Game is a draw","Good game man");
              for(int i=0;i<20;i++){
                show_face(14);
                ros::Duration(0.25).sleep();
                show_face(15);
                ros::Duration(0.25).sleep();
              }
              break;
      //Baxter loses
      case 3: send_gui_status("Baxter LOOSES","That cannot happen!!!! That is impossible, you cheated!!!!"); 
              for(int i=0;i<20;i++){
                show_face(16);
                ros::Duration(0.25).sleep();
                show_face(17);
                ros::Duration(0.25).sleep();
              }
              break;
    }

  }

  void show_face(int numofface){
    Mat face;
    int fieldnums=0;
    stringstream ss_num;
    stringstream ss_move;
    string path;
    //load the right face
    if(numofface>=100 && numofface<=135){
      
      ss_num << numofface;
      int temp=(numofface-100);
      ss_move << temp;
      fieldnums=numofface;
      numofface=100;
    }
      
    switch(numofface) {
      case 1 : path = ros::package::getPath("game_master") + "/faces/01_face_waiting.png";   // Read the file
               break;       // and exits the switch
      case 2 : path = ros::package::getPath("game_master") + "/faces/02_face_clean_board.png";   // Read the file
               break;       // and exits the switch
      case 3 : path = ros::package::getPath("game_master") + "/faces/03_face_game_start.png";   // Read the file
               break;       // and exits the switch
      case 4 : path = ros::package::getPath("game_master") + "/faces/04_face_ai_move.png";   // Read the file
               break;       // and exits the switch
      case 5 : path = ros::package::getPath("game_master") + "/faces/05_face_grasping_move.png";   // Read the file
               break;       // and exits the switc
      case 6 : path = ros::package::getPath("game_master") + "/faces/06_face_check_move.png";   // Read the file
               break;       // and exits the switch
      case 7 : path = ros::package::getPath("game_master") + "/faces/07_face_missed_move.png";   // Read the file
               break;       // and exits the switch
      case 8 : path = ros::package::getPath("game_master") + "/faces/08_face_thanks.png";   // Read the file
               break;       // and exits the switch
      case 9 : path = ros::package::getPath("game_master") + "/faces/09_face_human_move.png";   // Read the file
               break;       // and exits the switch
      case 10 : path = ros::package::getPath("game_master") + "/faces/10_face_human takes_long.png";   // Read the file
               break;       // and exits the switch
      case 11 : path = ros::package::getPath("game_master") + "/faces/11_face_human_done.png";   // Read the file
               break;       // and exits the switch
      case 12 : path = ros::package::getPath("game_master") + "/faces/12_face_baxter_win_a.png";   // Read the file
               break;       // and exits the switch
      case 13 : path = ros::package::getPath("game_master") + "/faces/13_face_baxter_win_b.png";   // Read the file
               break;       // and exits the switch
      case 14 : path = ros::package::getPath("game_master") + "/faces/14_face_draw_a.png";   // Read the file
               break;       // and exits the switch
      case 15 : path = ros::package::getPath("game_master") + "/faces/15_face_draw_b.png";   // Read the file
               break;       // and exits the switch
      case 16 : path = ros::package::getPath("game_master") + "/faces/16_face_baxter_loose_a.png";   // Read the file
               break;       // and exits the switch
      case 17 : path = ros::package::getPath("game_master") + "/faces/17_face_baxter_loose_b.png";   // Read the file
               break;       // and exits the switch
      case 20 : path = ros::package::getPath("game_master") + "/faces/20_face_baxter_mean.png";   // Read the file
               break;       // and exits the switch
      
      case 100 : path = ros::package::getPath("game_master") + "/faces/fieldnums/" + ss_num.str() + "_face_missed_move_" + ss_move.str()  +".png";   // Read the file
               break;       // and exits the switch
      
      case 200 : path = ros::package::getPath("game_master") + "/faces/200_face_baxter_countdown_0.png";   // Read the file
               break;       // and exits the switch
      case 201 : path = ros::package::getPath("game_master") + "/faces/201_face_baxter_countdown_1.png";   // Read the file
               break;       // and exits the switch
      case 202 : path = ros::package::getPath("game_master") + "/faces/202_face_baxter_countdown_2.png";   // Read the file
               break;       // and exits the switch
      case 203 : path = ros::package::getPath("game_master") + "/faces/203_face_baxter_countdown_3.png";   // Read the file
               break;       // and exits the switch
      case 204 : path = ros::package::getPath("game_master") + "/faces/204_face_baxter_countdown_4.png";   // Read the file
               break;       // and exits the switch
      case 205 : path = ros::package::getPath("game_master") + "/faces/205_face_baxter_countdown_5.png";   // Read the file
               break;       // and exits the switch
      
               
      
    }
    //read in face
    face = imread(path, CV_LOAD_IMAGE_COLOR);   // Read the file
    
    //check if image is ok
    if(! face.data ){                              // Check for invalid input
      ROS_ERROR_NAMED("game_master", "Could not open or find the face at: ");
      cout << path << endl;
      return ;
    }
    //displaz face 
        cv_bridge::CvImage out_msg;
        out_msg.header.frame_id   = "/world";//cv_ptr->header; // Same timestamp and tf frame as input image
        out_msg.header.stamp =ros::Time::now(); // new timestamp
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // encoding, might need to try some diffrent ones
        out_msg.image    = face; 
        image_pub_face_.publish(out_msg.toImageMsg()); //transfer to Ros image message 
      
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
    cout << "move detector got: baxter pieces: " << count_baxter_p << " human pieces: " << count_human_p << endl; 
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



  void request_grasping_baxter(int move,int pieces)
  {
    //Send which move should be executed
    game_master::grasping_baxter_game_masterGoal goal;
    goal.move = move;
    goal.pieces = pieces;

    // Need boost::bind to pass in the 'this' pointer
    ac_grasping_baxter.sendGoal(goal,
                boost::bind(&game_master_boss::received_grasping_done, this, _1, _2),
                actionlib::SimpleActionClient<game_master::grasping_baxter_game_masterAction>::SimpleActiveCallback(),
                boost::bind(&game_master_boss::received_grasping_feedback, this, _1));

  }

  void received_grasping_done(const actionlib::SimpleClientGoalState& state,
              const game_master::grasping_baxter_game_masterResultConstPtr& result)
  {
    //ROS_INFO("Finished in state [%s]", state.toString().c_str());
    //ROS_INFO("Answer: %i", result->sequence.back());
    //ROS_INFO("got move: ");
  }

  void received_grasping_feedback(const game_master::grasping_baxter_game_masterFeedbackConstPtr& feedback)
  {
    if(feedback->progress==77){
      ROS_DEBUG_NAMED("game_master","Recieved correct feedback");
      feedback_grasping=true;
    }
    else{
      ROS_ERROR_NAMED("game_master","Recieved wrong feedback.");

    }
    
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
    // publish info to the console for the user
    ROS_DEBUG_NAMED("game_master", "%s: start received", action_name_.c_str());
	
    //check if what button was pressed
    if(goal->command==1){
      stop_game=false;
      gui_start_flag=true;
    }
      
    if(goal->command==2){
      gui_start_flag=false;
      stop_game=true;
    }
    if(goal->command==100){
      gui_ground_state_flag=true;
    }
      
    if(goal->command==200){
      gui_check_reach_flag=true;
    }
  
  ROS_INFO("First player: %i", goal->first_player);
    //check first player
    if(goal->first_player ==2)
      baxter_starts=true;
    if(goal->first_player==1)
      baxter_starts=false;
    

    result_gui.game_started = 1;
    ROS_DEBUG_NAMED("game_master", "%s: Done", action_name_.c_str());
    // set the action state to succeeded
    as_gui.setSucceeded(result_gui);    
  }


  void send_gui_status(std::string status, std::string baxter_says)
  {
    //Send the gameboard *******
    //ROS_INFO("Sending away the status====");
    game_master::game_master_guiGoal goal;
    goal.status = status;
    goal.baxter_says=baxter_says;
    // Need boost::bind to pass in the 'this' pointer
    ac_guistatus.sendGoal(goal,
                boost::bind(&game_master_boss::received_gui_feedback, this, _1, _2),
                actionlib::SimpleActionClient<game_master::game_master_guiAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<game_master::game_master_guiAction>::SimpleFeedbackCallback());

  }



  void received_gui_feedback(const actionlib::SimpleClientGoalState& state,
              const game_master::game_master_guiResultConstPtr& result)
  {
    ROS_DEBUG("Gui Feedback receved");
    //ROS_INFO("Finished in state [%s]", state.toString().c_str());
    //ai_move =result->best_move;
    //std::cout << "best move is: " << ai_move << std::endl;
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
    //send first words
    gmb.game_pieces_baxter=18;
    
    while(ros::ok() && !gmb.gui_start_flag){
      gmb.send_gui_status("Waiting for command.","What's up man! When do we play?");
      ROS_INFO_THROTTLE(5, "Waiting for GUI-command");
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      gmb.show_face(1);
      if(gmb.gui_ground_state_flag){
        //order special grasping move ground state
        gmb.request_grasping_baxter(100,100);
        gmb.gui_ground_state_flag=false;
      }
      if(gmb.gui_check_reach_flag){
        //order special grasping move check reach
        gmb.request_grasping_baxter(200,200);
        gmb.gui_check_reach_flag=false;
      }
    }
    //select player and prepare board
    //init gameboard 
    gmb.gameboard.clear();
    for (int i = 0; i < 36; ++i)
    {
      gmb.gameboard.push_back(3);
    }
    //player 2 starts (baxtre)
    if(gmb.baxter_starts)
      gmb.gameboard.push_back(gmb.baxter_piece);
    else
      gmb.gameboard.push_back(gmb.baxter_piece);

    //check if gameboard is empty
    gmb.send_gui_status("Checking gameboard ...","Let's se if we can play.");
      
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
      gmb.send_gui_status("Gamboard not empty","Clean the board first!!");
      gmb.show_face(2);
      ROS_WARN("No empty Gamboard!! please clean up and try again.");
      ros::Duration(3.0).sleep();
      goto start_place;
    }
    

    //The game can finnaly start
    //print the gameboard
    gmb.print_gameboard();

    int EOG=false;
    //MAIN game LOOP+++++++++++++++++
    bool baxter_s=gmb.baxter_starts;
    
    gmb.send_gui_status("Game start","Let's gooooo");
    gmb.show_face(3);
    ros::Duration(3.0).sleep();
      
    while(ros::ok() && !EOG)
    {
      //check for stop
      if(gmb.stop_game)
        goto start_place;
      if(baxter_s){
        gmb.send_gui_status("Baxter AI","Let me think about a good move...");
        gmb.show_face(4);
    
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

        gmb.send_gui_status("Baxter Grasping","Look at this cool move");
        gmb.show_face(5);
    
        // send the move to the baxter_grasping
        gmb.request_grasping_baxter(gmb.ai_move,gmb.game_pieces_baxter);
        
        //wait for the grasping node to performe the pick place taask

        //faster!!!
        /*
        if(gmb.wait_for_result_grasping()){
          ROS_INFO("Pick place performet");
        }    
        else{
          ROS_ERROR("Failed to pick place in time.");
        }
        */

        //wait for the place feedback
        while(ros::ok() && !gmb.feedback_grasping){
          ROS_INFO_THROTTLE(5,"Waiting for feedback");
          ros::spinOnce();
          ros::Duration(0.1).sleep();
        }
        gmb.feedback_grasping=false;
        

        //we check if baxter succesfully placed the piece
        gmb.send_gui_status("Baxter checking move","Did I manage to do it?");
        gmb.show_face(6);
    
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
          //perform camera update
          gmb.request_update_camera(next_player);
          if(gmb.wait_for_result_camera()){
            ROS_INFO("Camera update performed with status: %i",gmb.camera_status); 
          }      
          else{
            ROS_ERROR("Failed to performe camera update n time");
          }

          ros::Duration(0.2).sleep();
          baxter_needs_help_counter++;

          // Baxter missed to place piece -> asks human to place it
          if(baxter_needs_help_counter>30){
            std::ostringstream baxy;
            baxy << "It seems I missed it ... can you move my red piece to field num: " << gmb.ai_move;
            gmb.send_gui_status("Baxter move failed", baxy.str());
            gmb.show_face(7+93+gmb.ai_move);
            ROS_WARN("Baxter missed the target. Set piece to: %i",gmb.ai_move);        
          }
            }
        if(baxter_needs_help_counter>30)
        {
          gmb.send_gui_status("Baxter move success","Thanks a lot!!");
          gmb.show_face(8);    
        }
        gmb.game_pieces_baxter--;
          

        //print the game board
        gmb.print_gameboard();

        //check for EOG
        if(isEOG(gmb.gameboard)){
          ROS_INFO("THE GAME IS OVER");
          if(playerXwin(gmb.gameboard,2)){
            ROS_INFO("BAXTER WINS!!! ");
            gmb.eog_reaction(1);
          }
          if(playerXwin(gmb.gameboard,1)){
            ROS_INFO("HUMAN WINS??? WTF!!!! ");
            gmb.eog_reaction(3);
          }
          if(isdraw(gmb.gameboard)){
            ROS_INFO("draw");
            gmb.eog_reaction(2);
          }
          EOG=true;
          break;
        }

      }
      ros::Duration(1.0).sleep();
      //make sure Baxter can play next time
      baxter_s=true;

      //check for stop
      gmb.send_gui_status("Human move","Do your best!!!"); 
      gmb.show_face(9); 
      if(gmb.stop_game)
        goto start_place;
      //Now we wait for the player to make a move
      if(gmb.gameboard[36]==1)
        next_player=1;
      if(gmb.gameboard[36]==2)
        next_player=2;
      int human_too_slow_counter=5;
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
        ros::Duration(1.1).sleep();

        //count down from 5
        if(human_too_slow_counter==5){
          ROS_INFO("Only 5 sec left for human!");
          gmb.send_gui_status("Human move ... still waiting","Only 5 sec left for human!");
          gmb.show_face(205);  
        }
        if(human_too_slow_counter==4){
          ROS_INFO("Only 4 sec left for human!");
          gmb.send_gui_status("Human move ... still waiting","Only 4 sec left for human!");
          gmb.show_face(204);  
        }
        if(human_too_slow_counter==3){
          ROS_INFO("Only 3 sec left for human!");
          gmb.send_gui_status("Human move ... still waiting","Only 3 sec left for human!");
          gmb.show_face(203); 
          //make noise 
        }
        if(human_too_slow_counter==2){
          ROS_INFO("Only 2 sec left for human!");
          gmb.send_gui_status("Human move ... still waiting","Only 2 sec left for human!");
          gmb.show_face(202);  
          //make noise 
        }
        if(human_too_slow_counter==1){
          ROS_INFO("Only 1 sec left for human!");
          gmb.send_gui_status("Human move ... still waiting","Only 2 sec left for human!");
          gmb.show_face(201);  
          //make noise 
        }
        if(human_too_slow_counter<=0){
          ROS_INFO("Human to Slow!!");
          gmb.send_gui_status("Human move ... still waiting","Human to Slow!!");
          gmb.show_face(20);  
          //make noise 
        }

        //decrese it
        human_too_slow_counter--;
          


      }
      if(human_too_slow_counter<=0){
        ROS_INFO("Human was to slow, bonus point for me!");
        gmb.send_gui_status("Human move done","Human was to slow, bonus point for me!");
        gmb.show_face(20); 
      }
      if(human_too_slow_counter>0){
        ROS_INFO("Nice!!"); 
        gmb.show_face(11); 
      }
          

     
      //print the game board
      gmb.print_gameboard();  

      //check for EOG
        if(isEOG(gmb.gameboard)){
          ROS_INFO("THE GAME IS OVER");
          if(playerXwin(gmb.gameboard,2)){
            ROS_INFO("BAXTER WINS!!! ");
            gmb.eog_reaction(1);
          }
          if(playerXwin(gmb.gameboard,1)){
            ROS_INFO("HUMAN WINS??? WTF!!!! ");
            gmb.eog_reaction(3);
          }
          if(isdraw(gmb.gameboard)){
            ROS_INFO("draw");
            gmb.eog_reaction(2);
          }
          EOG=true;
          break;
        }
        //fast strat
        if(gmb.wait_for_result_grasping()){
          ROS_INFO("Pick place performet");
        }    
        else{
          ROS_ERROR("Failed to pick place in time.");
        }

      ros::Duration(0.2).sleep();
      ros::spinOnce();

    }

    ros::spinOnce();
  }


  return 0;
}


