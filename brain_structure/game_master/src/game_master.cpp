#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <game_master/ai_game_masterAction.h> 
#include <game_master/gui_game_masterAction.h>
#include <game_master/grasping_baxter_game_masterAction.h> // links action file 
#include <game_master/camera_game_masterAction.h>
#include <game_master/watch_dog_game_masterAction.h>

using namespace std;

class game_master_boss
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<game_master::gui_game_masterAction> as_gui; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionClient<game_master::ai_game_masterAction> ac_ai;
  actionlib::SimpleActionClient<game_master::grasping_baxter_game_masterAction> ac_grasping_baxter;
  actionlib::SimpleActionClient<game_master::camera_game_masterAction> ac_camera;
  actionlib::SimpleActionClient<game_master::watch_dog_game_masterAction> ac_watch_dog;

  std::string action_name_;
  // create messages that are used to published feedback/result
  game_master::gui_game_masterFeedback feedback_gui; // create messages that are used to published feedback
  game_master::gui_game_masterResult result_gui;    // create messages that are used to published result



public:
  bool gui_start_flag;
  bool ai_received_move_flag;
  bool grasping_done_flag;
  bool camera_done_flag;
  bool watch_dog_done_flag;
  int ai_move;
  std::vector<int> gameboard;

  game_master_boss(std::string name) :
    as_gui(nh_, name, boost::bind(&game_master_boss::gui_start_command, this, _1), false),
    action_name_(name), ac_ai("ai_game_master", true), ac_grasping_baxter("grasping_baxter_game_master", true), ac_camera("camera_game_master", true),ac_watch_dog("watch_dog_game_master", true), gui_start_flag(false),ai_received_move_flag(false),grasping_done_flag(false)
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

    ROS_INFO("looking for  watch dog server server.");
    ac_watch_dog.waitForServer(); // wait for ai server to be active
    ROS_INFO("found wach dog server.");

  }

  ~game_master_boss(void)
  {
  }
 //TODO: send whole gamboard
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
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    //ROS_INFO("Answer: %i", result->sequence.back());
    ai_move =result->best_move;
    std::cout << "best move is: " << ai_move << std::endl;
    ai_received_move_flag=true;
  }


void request_update_camera(int update)
  {
    //Send the gmaeboard *******
    game_master::camera_game_masterGoal goal;
    goal.update = update;

    // Need boost::bind to pass in the 'this' pointer
    ac_camera.sendGoal(goal,
                boost::bind(&game_master_boss::received_update_camera, this, _1, _2),
                actionlib::SimpleActionClient<game_master::camera_game_masterAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<game_master::camera_game_masterAction>::SimpleFeedbackCallback());

  }


  void received_update_camera(const actionlib::SimpleClientGoalState& state,
              const game_master::camera_game_masterResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    //ROS_INFO("Answer: %i", result->sequence.back());
    //get new gameboard here!!!
    gameboard=result->gameboard;
    //TODO: check if only one piece and other stuff
    ROS_INFO("update_camera_done ");
    camera_done_flag=true;
  }



void request_watch_dog(int start_watch_dog)
  {
    //Send the gmaeboard *******
    game_master::watch_dog_game_masterGoal goal;
    goal.start_watch_dog = start_watch_dog;

    // Need boost::bind to pass in the 'this' pointer
    ac_watch_dog.sendGoal(goal,
                boost::bind(&game_master_boss::received_watch_dog, this, _1, _2),
                actionlib::SimpleActionClient<game_master::watch_dog_game_masterAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<game_master::watch_dog_game_masterAction>::SimpleFeedbackCallback());

  }


  void received_watch_dog(const actionlib::SimpleClientGoalState& state,
              const game_master::watch_dog_game_masterResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    //ROS_INFO("Answer: %i", result->sequence.back());
    ROS_INFO("watch_dog_done ");
    watch_dog_done_flag=true;
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
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    //ROS_INFO("Answer: %i", result->sequence.back());
    ROS_INFO("got move: ");
    grasping_done_flag=true;
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
  //start action server
  game_master_boss gmb("gui_game_master");

  while(ros::ok() && !gmb.gui_start_flag){
    ROS_INFO_THROTTLE(5, "Waiting for GUI");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  ROS_INFO_THROTTLE(1, "GUI OK");
  
  //init gameboard
  gmb.gameboard.clear();
  for (int i = 0; i < 49; ++i)
  {
    gmb.gameboard.push_back(0);
  }

  
int EOG=false;
  //MAIN LOOP+++++++++++++++++

while(ros::ok() && !EOG)
{
  //reset flags
  gmb.ai_received_move_flag=false;
  gmb.grasping_done_flag=false;
  gmb.camera_done_flag=false;
  gmb.watch_dog_done_flag=false;


  // ask AI for a cool move 
  gmb.request_move_ai(gmb.gameboard);

  while(ros::ok() && !gmb.ai_received_move_flag){
    ROS_INFO_THROTTLE(5, "Waiting for Ai");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  ROS_INFO_THROTTLE(1, "Ai ok");

  // send the cool move to the baxter_grasping
  gmb.request_grasping_baxter(gmb.ai_move);

  while(ros::ok() && !gmb.grasping_done_flag){
    ROS_INFO_THROTTLE(5, "Waiting for move ");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  ROS_INFO_THROTTLE(1, "move done");

  //ask camera to perform an update

  int update=1;
  gmb.request_update_camera(update);

  while(ros::ok() && !gmb.camera_done_flag ){
    ROS_INFO_THROTTLE(5, "Waiting for camera ");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  gmb.camera_done_flag=false;

  ROS_INFO_THROTTLE(1, "camera done");
  //Print gameboard
  int rows=6;
  int cols=6;
  cout << "*********************************" << endl;
  for(int i=0;i<rows;i++){
    for(int j=0;j<cols;j++){
      cout << "|" << gmb.gameboard[j+rows*i];
    }
    cout << "|" << endl;
    cout << "_______________" << endl;    
  }
  cout << "*********************************" << endl;


  // the plazer is on
  int wuff=1;
  gmb.request_watch_dog(wuff);

  while(ros::ok() && !gmb.watch_dog_done_flag ){
    ROS_INFO_THROTTLE(5, "Waiting for watchdog ");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  ROS_INFO_THROTTLE(1, "wauzi done");

  //player done


  // ask camera for update
  update=1;
  gmb.request_update_camera(update);

  while(ros::ok() && !gmb.camera_done_flag ){
    ROS_INFO_THROTTLE(5, "Waiting for camera ");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  ROS_INFO_THROTTLE(1, "camera done");


  //Print gameboard
  cout << "*********************************" << endl;
  for(int i=0;i<rows;i++){
    for(int j=0;j<cols;j++){
      cout << "|" << gmb.gameboard[j+rows*i];
    }
    cout << "|" << endl;
    cout << "_______________" << endl;    
  }
  cout << "*********************************" << endl;

  //*************completed one circle

  //TODO:check for EOG and stuff



}

  

  return 0;
}