//************************************************
//**********Baxter 2017 Tic-Tac-Toe***************
//*******Nadine Drollinger & Michael C.Welle********
//************************************************
//*******Grasping node - grasping_baxter *********
//************************************************

//************************************************
//Description: gets the ar-tag postioin and the 
// field number. Calculates pick and place poses 
// and does it.
//************************************************

//ros
#include <ros/ros.h>
#include <ros/console.h>
//action
#include <actionlib/server/simple_action_server.h>
#include <grasping_baxter/grasping_baxter_game_masterAction.h>
//moveit
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
//endeffector control
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/DigitalIOState.h>
//tf
#include  <tf/transform_datatypes.h>
//ar-tag tracker
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
//opencv
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//c++
#include <iostream>
#include <stdio.h>
#include <math.h>

using namespace cv;
using namespace std;

//grasping class
class grasping_baxter_boss
{
protected:
  //nodehandler
  ros::NodeHandle nh_;
  //action initializing
  actionlib::SimpleActionServer<grasping_baxter::grasping_baxter_game_masterAction> as_grasping_baxter; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  grasping_baxter::grasping_baxter_game_masterFeedback feedback_grasping_baxter; // create messages that are used to published feedback
  grasping_baxter::grasping_baxter_game_masterResult result_grasping_baxter;    // create messages that are used to published result

public:
  //vars
	bool debug_flag;
  bool doarupdate;
  int target_field;
  int gripperSeq;
  int num_game_pieces_left;
  int art_vec_count;
  int art_vec_position;
  int current_stack;
  int num_baxtertrys;
  //moveit groups and interfaces
  moveit::planning_interface::MoveGroup group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroup::Plan my_plan;
  //publisher and subscriber
  ros::Publisher ar_pub;
  ros::Publisher target_pub;
  ros::Publisher pp_pub;
  ros::Publisher rightGripperPub;
  ros::Subscriber ar_pose_sub;
  //poses
  geometry_msgs::PoseStamped ar_code_pose;
  geometry_msgs::PoseStamped target_pose;
  geometry_msgs::PoseStamped p0_pose;
  geometry_msgs::PoseStamped pick_up_pose;
  
  // offset from ar to p0 (field(0)) 
  double offset_p0_pose_x;
  double offset_p0_pose_y;
  double offset_p0_pose_z;
  // offset for piece-box storage
  double offset_pick_up_pose_x;
  double offset_pick_up_pose_y;
  double offset_pick_up_pose_z;
  //x & y offset for field to field
  double offset_ff_x;
  double offset_ff_y;
  // x & y offset stack to stack
  double offset_ss_x;
  double offset_ss_y;
  // x & y offset for center pose
  double offset_bc_x;
  double offset_bc_y;
  double offset_bc_z;
  //ar-tag var
  std::vector<geometry_msgs::PoseStamped> ar_tag_pose_vector;

  
  //NOTE: this is where the hardware constant need to be entered
  //bord layout:
  //
  //--tag--tag---ar-pose---tag--tag--
  //__________________________________
  //
  //               |
  //               |x
  //               |
  //        <___y__|
  //
  grasping_baxter_boss(std::string name) :
    as_grasping_baxter(nh_, name, boost::bind(&grasping_baxter_boss::grasping_baxter_start_command, this, _1), false),
    action_name_(name),group("right_arm"),offset_p0_pose_x(-0.101),offset_p0_pose_y(+0.16),
    offset_p0_pose_z(0.015),offset_pick_up_pose_x(-0.32),offset_pick_up_pose_y(-0.275),offset_pick_up_pose_z(0.06),art_vec_count(0),
    art_vec_position(0),ar_tag_pose_vector(10),debug_flag(false),num_game_pieces_left(18),
    doarupdate(true), offset_bc_x(-0.24), offset_bc_y(0),offset_bc_z(0),current_stack(0),num_baxtertrys(3)
  {
    ROS_DEBUG_NAMED("grasping_baxter","Initilzing Grasping baxter boss");
    // start action server
    as_grasping_baxter.start();

    //poses pub and subscribers
    ar_pub = nh_.advertise<geometry_msgs::PoseStamped>("/TTTgame/poses/ar_code", 1);
    target_pub = nh_.advertise<geometry_msgs::PoseStamped>("/TTTgame/poses/target", 1);
    pp_pub = nh_.advertise<geometry_msgs::PoseStamped>("/TTTgame/poses/pick_place", 1);
    ar_pose_sub = nh_.subscribe("/TTTgame/ar_tag/ar_pose", 1, &grasping_baxter_boss::ar_code_pose_callback, this);
    rightGripperPub = nh_.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command",10, true);
    //calibrate gripper
    calibraterightGripper();
    configurerightGripper();
    //update enviroment
    grasping_baxter_environment_fixed();
    grasping_baxter_environment_dynamic();
  }

  //deconstructor
  ~grasping_baxter_boss(void)
  {
  }
  //configure gripper
  bool configurerightGripper()
  {
    ROS_DEBUG_NAMED("grasping_baxter", "Calibrate right arm gripper");
    baxter_core_msgs::EndEffectorCommand command;
    command.command = baxter_core_msgs::EndEffectorCommand::CMD_CONFIGURE;
    command.args = "{\"vacuum_sensor_threshold\":7.0}";
    command.id = 65537;
    command.sender = "grasping_baxter";
    command.sequence = gripperSeq++;   
    rightGripperPub.publish(command);        
    ros::Duration(0.2).sleep();
    return true;
  }

  //calibrate right gripper
  bool calibraterightGripper()
  {
    ROS_DEBUG_NAMED("grasping_baxter", "Calibrate right arm gripper");
    baxter_core_msgs::EndEffectorCommand command;
    command.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
    command.id = 65537;
    command.sender = "grasping_baxter";
    command.sequence = gripperSeq++;   
    rightGripperPub.publish(command);        
    ros::Duration(0.2).sleep();
    return true;
  }

  //close right gripper (sucking)
  bool closerightGripper()
  {
    ROS_DEBUG_NAMED("grasping_baxter", "Closing right arm gripper");
    baxter_core_msgs::EndEffectorCommand command;
    command.command = baxter_core_msgs::EndEffectorCommand::CMD_GRIP;
    command.id = 65537;
    command.sequence = gripperSeq++;
    command.sender = "grasping_baxter";
    command.args = "{\"grip_attempt_seconds\": 3.0}";
    //set_vacuum_threshold
    rightGripperPub.publish(command);    
    ros::Duration(0.1).sleep();
    return true;
  }

  //open right gripper (releasing)
  bool openrightGripper()
  {
    ROS_DEBUG_NAMED("grasping_baxter", "Opening right arm gripper");
    baxter_core_msgs::EndEffectorCommand command;
    command.command = baxter_core_msgs::EndEffectorCommand::CMD_RELEASE;
    command.id = 65537;
    command.sequence = gripperSeq++;
    command.sender = "grasping_baxter";
    rightGripperPub.publish(command);
    ros::Duration(0.1).sleep();
    return true;
  }

  void move_to_ground_pose(){
  //move with joint stuff to ground pos
    ROS_DEBUG_NAMED("grasping_baxter", "Moving to ground state.");
    std::string joint_names="'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2'";
    group.setJointValueTarget("right_e0",-0.08590292412158317);
    group.setJointValueTarget("right_e1",1.9987769666146942);
    group.setJointValueTarget("right_s0",-0.6212622190935926);
    group.setJointValueTarget("right_s1",-1.0841409218380162);
    group.setJointValueTarget("right_w0",0.11236409271260656);
    group.setJointValueTarget("right_w1",0.8053399136398423);
    group.setJointValueTarget("right_w2",-0.3896311201228951);
    group.plan(my_plan);
    bool success = group.execute(my_plan);
    if(success)
      ROS_DEBUG_NAMED("grasping_baxter", "Performed ground state move");
    else
      ROS_ERROR_NAMED("grasping_baxter", "Failed to reach ground state");
  }

  void check_reach(){
    ROS_DEBUG_NAMED("grasping_baxter", "performing reach check");
    int outerbound [21] = {0,1,2,3,4,5,11,17,23,29,35,34,33,32,31,30,24,18,12,6};
    for(int i=0;i<21;i++){
      //move to pick up pose    
      target_pose=pick_up_pose;
      target_pose.pose.position.z +=0.05; 
      bool success=false;
      group.setPoseTarget(target_pose);
      group.plan(my_plan);
      success = group.execute(my_plan);
      sleep(1.1);
      if(success)
        ROS_DEBUG_NAMED("grasping_baxter", "reached pick and place pose again");
      else
        ROS_ERROR_NAMED("grasping_baxter", "Failed to reach pick and place pose again");

      //move to outerbound field
      //calculate the field possition
      int row=outerbound[i]/6;
      int col=outerbound[i]%6;

      target_pose.pose.position.x=p0_pose.pose.position.x+row*offset_ff_x;
      target_pose.pose.position.y=p0_pose.pose.position.y+col*offset_ff_y;
      target_pose.pose.position.z=pick_up_pose.pose.position.z;
      ROS_DEBUG_NAMED("grasping_baxter","Move to field num: %i",target_field);
      success=false;
      group.setStartStateToCurrentState();
      group.setPoseTarget(target_pose);
      group.plan(my_plan);
      success = group.execute(my_plan);
      sleep(0.1);
      if(success)
        ROS_DEBUG_NAMED("grasping_baxter", "Reached place pose + some z margin.");
      else
        ROS_ERROR_NAMED("grasping_baxter", "Failed to reach place pose + some z margin.");
      
      sleep(1.1);

      // move down to p0 z pose
      target_pose.pose.position.z = p0_pose.pose.position.z;
      success=false;
      group.setStartStateToCurrentState();
      group.setPoseTarget(target_pose);
      group.plan(my_plan);
      success = group.execute(my_plan);
      sleep(0.1);
      if(success)
        ROS_DEBUG_NAMED("grasping_baxter", "Reached place pose down.");
      else
        ROS_ERROR_NAMED("grasping_baxter", "Failed to reach place pose down.");
      sleep(1.2);
      // move up again to p0 z pose
      target_pose.pose.position.z = pick_up_pose.pose.position.z+0.05;
      success=false;
      group.setStartStateToCurrentState();
      group.setPoseTarget(target_pose);
      group.plan(my_plan);
      success = group.execute(my_plan);
      sleep(0.1);
      if(success)
        ROS_DEBUG_NAMED("grasping_baxter", "Reached place pose up.");
      else
        ROS_ERROR_NAMED("grasping_baxter", "Failed to reach place pose up.");
      sleep(1.2);
    }

    //move to pick up pose    
      target_pose=pick_up_pose;
      target_pose.pose.position.z +=0.05; 
      bool success=false;
      group.setPoseTarget(target_pose);
      group.plan(my_plan);
      success = group.execute(my_plan);
      sleep(0.1);
      if(success)
        ROS_DEBUG_NAMED("grasping_baxter", "reached pick and place pose again");
      else
        ROS_ERROR_NAMED("grasping_baxter", "Failed to reach pick and place pose again");

    
  }

  //Callback from ar-tag, takes the avarage of 10 poses (in sliding avarage mode) 
  // and calculates pickup pose and p0 pose debending on it.
  void ar_code_pose_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
  {
    //only do a update when allowed
    if(!doarupdate)
      return;

    ROS_DEBUG_NAMED("grasping_baxter","performing ar-tag calculations");
    geometry_msgs::PoseStamped ar_code_7_pose;
    geometry_msgs::PoseStamped ar_code_13_pose;
    geometry_msgs::PoseStamped ar_code_10_pose;
    geometry_msgs::PoseStamped ar_code_0_pose;
    double theta;
    //check if there are 2 poses
    if(msg->markers.size()!=4){
      ROS_WARN_NAMED("grasping_baxter", "Not 4 Markers");
      return;
    }
    //check if there is a minimum x
    for(int i=0;i<msg->markers.size();i++){
      if(msg->markers[i].pose.pose.position.x<0.3){
        ROS_WARN_NAMED("grasping_baxter", "to close to be true");
        return;
      }
    }
    
    //loop through the 4 marker to get marker id
    //---7---13-----10---0---
    //_________________________   
    //|   |   |   |   |   |   |
    for(int i=0;i<msg->markers.size();i++){
      if(msg->markers[i].id==7){
        ar_code_7_pose=msg->markers[i].pose;
      }
      if(msg->markers[i].id==13){
        ar_code_13_pose=msg->markers[i].pose;
      }
      if(msg->markers[i].id==10){
        ar_code_10_pose=msg->markers[i].pose;
      }
      if(msg->markers[i].id==0){
        ar_code_0_pose=msg->markers[i].pose;
      }
    }

    //combine the 4 ar_code poses to one
    ar_code_pose.header.frame_id="/world";
    ar_code_pose.header.stamp = ros::Time::now();;
    // x=(7x+13x+10x+0x)/4
    ar_code_pose.pose.position.x=(ar_code_7_pose.pose.position.x+ar_code_13_pose.pose.position.x+ar_code_10_pose.pose.position.x+ar_code_0_pose.pose.position.x)/4;
    // y=((7y+13y+10y+0y)/4)
    ar_code_pose.pose.position.y=(ar_code_7_pose.pose.position.y+ar_code_13_pose.pose.position.y+ar_code_10_pose.pose.position.y+ar_code_0_pose.pose.position.y)/4;
    // z=((7z+13z+10z+0z)/4)
    ar_code_pose.pose.position.z=(ar_code_7_pose.pose.position.z+ar_code_13_pose.pose.position.z+ar_code_10_pose.pose.position.z+ar_code_0_pose.pose.position.z)/4;
    
    //TODO:fix kenect input
    //FIXED z because KINECT SUCKS!!!!!!
    ar_code_pose.pose.position.z=-0.161;
    //calculate yaw angel given x of the ar-tags
    double x_delta1=abs(ar_code_7_pose.pose.position.x-ar_code_0_pose.pose.position.x)/2;
    double x_delta2=abs(ar_code_13_pose.pose.position.x-ar_code_10_pose.pose.position.x)/2;
    if(ar_code_7_pose.pose.position.x>ar_code_0_pose.pose.position.x){
      theta = 1.5*M_PI+(atan(0.173/x_delta1)+atan(0.065/x_delta2))/2;
    }
    if(ar_code_7_pose.pose.position.x<ar_code_0_pose.pose.position.x){
      theta = (atan(x_delta1/0.173)+atan(x_delta2/0.065))/2;
    }
    if(ar_code_7_pose.pose.position.x=ar_code_0_pose.pose.position.x){
      theta = 0.0;
    }
    
    ar_code_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    //sliding average calculation for ar-tag pose
    ar_tag_pose_vector[art_vec_position]=ar_code_pose;
    art_vec_position++;
    if(art_vec_position>=10)
      art_vec_position=0;

    geometry_msgs::PoseStamped art_pose_temp=ar_tag_pose_vector[0];
    if(art_vec_count<10)
      art_vec_count++;

    for(int i=1;i<art_vec_count;i++){
      art_pose_temp.pose.position.x+=ar_tag_pose_vector[i].pose.position.x;
      art_pose_temp.pose.position.y+=ar_tag_pose_vector[i].pose.position.y;
      art_pose_temp.pose.position.z+=ar_tag_pose_vector[i].pose.position.z;
      art_pose_temp.pose.orientation.x+=ar_tag_pose_vector[i].pose.orientation.x;
      art_pose_temp.pose.orientation.y+=ar_tag_pose_vector[i].pose.orientation.y;
      art_pose_temp.pose.orientation.z+=ar_tag_pose_vector[i].pose.orientation.z;
      art_pose_temp.pose.orientation.w+=ar_tag_pose_vector[i].pose.orientation.w;      
    }

    ar_code_pose.pose.position.x=art_pose_temp.pose.position.x/art_vec_count;
    ar_code_pose.pose.position.y=art_pose_temp.pose.position.y/art_vec_count;
    ar_code_pose.pose.position.z=art_pose_temp.pose.position.z/art_vec_count;
    ar_code_pose.pose.orientation.x=art_pose_temp.pose.orientation.x/art_vec_count;
    ar_code_pose.pose.orientation.y=art_pose_temp.pose.orientation.y/art_vec_count;
    ar_code_pose.pose.orientation.z=art_pose_temp.pose.orientation.z/art_vec_count;
    ar_code_pose.pose.orientation.w=art_pose_temp.pose.orientation.w/art_vec_count;

    //update the enviroment with the ar_code pose
    //TODO:Fix dynamic obsticals
    //grasping_baxter_environment_dynamic();
    // calculate p0 pose using x and y offset (known)
    theta=tf::getYaw(ar_code_pose.pose.orientation);
    p0_pose.header.stamp=ros::Time::now();
    p0_pose.header.frame_id="/world";
    // calc P0 with ar at origin
    Mat p0 = (Mat_<double>(2,1) <<
                    offset_p0_pose_x,
                    offset_p0_pose_y);
    // rotate around theta
    Mat r = (Mat_<double>(2,2) <<
                    cos(theta), -sin(theta),
                    sin(theta), cos(theta));
    Mat p0r= r*p0;
    //translate to get true pose
    Mat t = (Mat_<double>(2,1)  <<
                    ar_code_pose.pose.position.x,
                    ar_code_pose.pose.position.y);
    p0=p0r+t;
    //set it to p0 pose (x,y)
    p0_pose.pose.position.x=p0.at<double>(0,0);
    p0_pose.pose.position.y=p0.at<double>(1,0);
    //set z
    p0_pose.pose.position.z=ar_code_pose.pose.position.z+offset_p0_pose_z;
    //set orientation
    //NOTE: not sure why baxter need this orientation to grasp from the top ...
    p0_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,0,1.57);
    //calculate offsets to other P (p1-58)
    t = (Mat_<double>(2,1)  <<
                    -0.065,
                    -0.065);
    t = r*t;
    offset_ff_x =  t.at<double>(0,0);
    offset_ff_y =  t.at<double>(1,0);
    //calculate pickup pose  dependig on how many pieces are left
    pick_up_pose.header.stamp=ros::Time::now();
    pick_up_pose.header.frame_id="/world";
    // calc pick up pose with ar at origin
    Mat ppu = (Mat_<double>(2,1) <<
                    offset_pick_up_pose_x,
                    offset_pick_up_pose_y);
    // rotate around theta
     r = (Mat_<double>(2,2) <<
                    cos(theta), -sin(theta),
                    sin(theta), cos(theta));
    Mat ppur= r*ppu;
    //translate to get true pose
     t = (Mat_<double>(2,1)  <<
                    ar_code_pose.pose.position.x,
                    ar_code_pose.pose.position.y);
    ppu=ppur+t;
    //set it to p0 pose (x,y)
    pick_up_pose.pose.position.x=ppu.at<double>(0,0);
    pick_up_pose.pose.position.y=ppu.at<double>(1,0);
    //set z
    pick_up_pose.pose.position.z=ar_code_pose.pose.position.z+offset_pick_up_pose_z;
    //set oriantion
    //NOTE: not sure why baxter need this orientation to grasp from the top ...
    pick_up_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,0,1.57);
     
  }
  
  //publishing poses
  void publish_poses(){
  	ar_pub.publish(ar_code_pose);    
    target_pub.publish(target_pose);
    pp_pub.publish(pick_up_pose);
  }

  void pick_up_firs_piece(){
    ROS_DEBUG_NAMED("grasping_baxter", "Performing Pick up the first piece.");
    //define target pose
    target_pose.header.stamp=ros::Time::now();
    target_pose.header.frame_id="/world";
    target_pose=pick_up_pose;

    //move to pick up pose + some z margin
    target_pose.pose.position.z +=0.05;    
    bool success=false;
    int moveitcounter=0;
    group.setStartStateToCurrentState();
    while(!success && moveitcounter <num_baxtertrys){
      group.setPoseTarget(target_pose);
      group.plan(my_plan);
      success = group.execute(my_plan);
      sleep(0.1);
      if(success)
        ROS_DEBUG_NAMED("grasping_baxter", "Reached pick_up_pose + some margin");
      else
        ROS_ERROR_NAMED("grasping_baxter", "Failed to reach pick_up_pose + some margin");
      ros::spinOnce();
      moveitcounter++;
    }
    if(moveitcounter>=num_baxtertrys){
      ROS_ERROR_NAMED("grasping_baxter", "FATAL ERROR: Failed to reach pick_up_pose + some margin 5x");
    }
       
    //move down following a cartesian path
    geometry_msgs::Pose car_target_pose;
    //translate into pose
    car_target_pose.position=pick_up_pose.pose.position;
    car_target_pose.orientation= pick_up_pose.pose.orientation;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.clear();
    car_target_pose.position.z+=0.01;
    //add more points depending on how many pieces are left
    waypoints.push_back(car_target_pose);
    car_target_pose.position.z-=0.01;
    waypoints.push_back(car_target_pose);
    
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints,
                                             0.002,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory);
      
    
    success=false;
    moveitcounter=0;
    group.setStartStateToCurrentState();
    while(!success && moveitcounter <num_baxtertrys){
      my_plan.trajectory_ = trajectory;
      success = group.execute(my_plan);
      sleep(0.1);
      if(success)
        ROS_DEBUG_NAMED("grasping_baxter", "Reached Pick up pose for piece.");
      else
        ROS_ERROR_NAMED("grasping_baxter", "Failed to reach Pick up pose for piece.");
      ros::spinOnce();
      moveitcounter++;
    }
    if(moveitcounter>=num_baxtertrys){
      ROS_ERROR_NAMED("grasping_baxter", "FATAL ERROR: Failed to reach Pick up pose for piece. 5x");
    }
    
    sleep(0.5);
    //suck up the piece
    closerightGripper();
    sleep(0.8);

    ROS_DEBUG_NAMED("grasping_baxter", "Pieces left in game: %i",num_game_pieces_left);  
    
    //move up again 
    //move up following a cartesian path
    waypoints.clear();
    waypoints.push_back(car_target_pose);
    
    car_target_pose.position.z += 0.05;
    waypoints.push_back(car_target_pose);
    fraction = group.computeCartesianPath(waypoints,
                                             0.002,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory);
    
    
    success=false;
    moveitcounter=0;
    group.setStartStateToCurrentState();
    while(!success && moveitcounter <num_baxtertrys){
      my_plan.trajectory_ = trajectory;
      success = group.execute(my_plan);
      sleep(0.1);
      if(success)
        ROS_DEBUG_NAMED("grasping_baxter", "Reached pick_up_pose + some margin with a piece");
      else
        ROS_ERROR_NAMED("grasping_baxter", "Failed to reach pick_up_pose + some margin with a piece");
      ros::spinOnce();
      moveitcounter++;
    }
    if(moveitcounter>=num_baxtertrys){
      ROS_ERROR_NAMED("grasping_baxter", "FATAL ERROR: Failed to reach pick_up_pose + some margin with a piece. 5x");
    }
    sleep(0.1);

  }

  //function to pick up a piece and place it where the Ai wants it
  void place_piece(){
    ROS_DEBUG_NAMED("grasping_baxter", "Performing Pick and Place.");
   //define target pose
    target_pose.header.stamp=ros::Time::now();
    target_pose.header.frame_id="/world";
    target_pose=pick_up_pose;

    //move to place pos with the pick up z  
    //calculate the field possition
    int row=target_field/6;
    int col=target_field%6;

    target_pose.pose.position.x=p0_pose.pose.position.x+row*offset_ff_x;
    target_pose.pose.position.y=p0_pose.pose.position.y+col*offset_ff_y;
    target_pose.pose.position.z=pick_up_pose.pose.position.z;
    ROS_DEBUG_NAMED("grasping_baxter","Move to field num: %i",target_field);
    bool success=false;
    int moveitcounter=0;
    group.setStartStateToCurrentState();
    while(!success && moveitcounter <num_baxtertrys){
      group.setPoseTarget(target_pose);
      group.plan(my_plan);
      success = group.execute(my_plan);
      sleep(0.1);
      if(success)
        ROS_DEBUG_NAMED("grasping_baxter", "Reached place pose + some z margin.");
      else
        ROS_ERROR_NAMED("grasping_baxter", "Failed to reach place pose + some z margin.");
      ros::spinOnce();
      moveitcounter++;
    }
    if(moveitcounter>=num_baxtertrys){
      ROS_ERROR_NAMED("grasping_baxter", "FATAL ERROR: Failed to reach pick_up_pose + some margin with a piece. 5x");
    }
    
    sleep(0.1);

    // move down to p0 z pose
    target_pose.pose.position.z = p0_pose.pose.position.z;
    success=false;
    moveitcounter=0;
    group.setStartStateToCurrentState();
    while(!success && moveitcounter <num_baxtertrys){
      group.setPoseTarget(target_pose);
      group.plan(my_plan);
      success = group.execute(my_plan);
      sleep(0.1);
      if(success)
        ROS_DEBUG_NAMED("grasping_baxter", "Reached place pose.");
      else
        ROS_ERROR_NAMED("grasping_baxter", "Failed to reach place pose.");
      ros::spinOnce();
      moveitcounter++;
    }
    if(moveitcounter>=num_baxtertrys){
      ROS_ERROR_NAMED("grasping_baxter", "FATAL ERROR: Failed to reach place pose. 5x");
    }
    
    sleep(0.1);
    //release the piece
    openrightGripper();
    sleep(0.1);
    //send feedback that we placed the piece
    feedback_grasping_baxter.progress=77;
    as_grasping_baxter.publishFeedback(feedback_grasping_baxter);

    //move up again
    target_pose.pose.position.z=pick_up_pose.pose.position.z+0.05;
    success=false;
    moveitcounter=0;
    group.setStartStateToCurrentState();
    while(!success && moveitcounter <num_baxtertrys){
      group.setPoseTarget(target_pose);
      group.plan(my_plan);
      success = group.execute(my_plan);
      sleep(0.1);
      if(success)
        ROS_DEBUG_NAMED("grasping_baxter", "Reached place pose + some margin after dropping piece");
      else
        ROS_ERROR_NAMED("grasping_baxter", "Failed to reach place pose + some margin after dropping piece");
      ros::spinOnce();
      moveitcounter++;
    }
    if(moveitcounter>=num_baxtertrys){
      ROS_ERROR_NAMED("grasping_baxter", "FATAL ERROR: Failed to reach place pose + some margin after dropping piece 5x");
    }    
    sleep(0.1);

    //check how many pieces
    int pieces_in_stack=num_game_pieces_left % 6;
    bool new_stack=false;
    if(num_game_pieces_left % 6==0)
    {
     pieces_in_stack=6;
     if(num_game_pieces_left<18)
       new_stack=true; 
    }

    // it is one less
    pieces_in_stack--;
    if(pieces_in_stack<=0)
      new_stack=true; 
    num_game_pieces_left--;

    //move to pick up pose    
    target_pose=pick_up_pose;
    target_pose.pose.position.z +=0.05; 
    //temp update the stack?
    if(new_stack){
      new_stack=false;
      double theta=tf::getYaw(ar_code_pose.pose.orientation);
      Mat r = (Mat_<double>(2,2) <<
                      cos(theta), -sin(theta),
                      sin(theta), cos(theta));
      //translate to get true pose
      Mat t = (Mat_<double>(2,1)  <<
                      0.0625,
                      0.0);
      t = r*t;
      offset_ss_x =  t.at<double>(0,0);
      offset_ss_y =  t.at<double>(1,0);
      //second stack
      if(num_game_pieces_left<13 && current_stack==0){
        pick_up_pose.pose.position.x+=offset_ss_x;
        pick_up_pose.pose.position.y+=offset_ss_y;
      }
      //third stack
      if(num_game_pieces_left<7 && current_stack==1){
        pick_up_pose.pose.position.x+=offset_ss_x;
        pick_up_pose.pose.position.y+=offset_ss_y;
      }
      pieces_in_stack=6;
    }

    success=false;
    moveitcounter=0;
    while(!success && moveitcounter <num_baxtertrys){
      group.setPoseTarget(target_pose);
      group.plan(my_plan);
      success = group.execute(my_plan);
      sleep(0.1);
    if(success)
      ROS_DEBUG_NAMED("grasping_baxter", "reached pick and place pose again");
    else
      ROS_ERROR_NAMED("grasping_baxter", "Failed to reach pick and place pose again");
    ros::spinOnce();
    moveitcounter++;
    }
    if(moveitcounter>=num_baxtertrys){
      ROS_ERROR_NAMED("grasping_baxter", "FATAL ERROR: Failed to reach place pose + some margin after dropping piece 5x");
    }
    sleep(0.2);
    //down again depending on how many pieces
    //move down following a cartesian path
    geometry_msgs::Pose car_target_pose;
    //translate into pose
    car_target_pose.position=pick_up_pose.pose.position;
    car_target_pose.orientation= pick_up_pose.pose.orientation;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.clear();
    car_target_pose.position.z+=0.01;
    //add more points depending on how many pieces are left
    waypoints.push_back(car_target_pose);
    car_target_pose.position.z-=0.01;
    waypoints.push_back(car_target_pose);
    for(int i =0; i<6-pieces_in_stack;i++){
      car_target_pose.position.z -= 0.01;
      waypoints.push_back(car_target_pose);
    }
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints,
                                             0.002,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory);
      
    
    success=false;
    moveitcounter=0;
    group.setStartStateToCurrentState();
    while(!success && moveitcounter <num_baxtertrys){
      my_plan.trajectory_ = trajectory;
      success = group.execute(my_plan);
      sleep(0.1);
      if(success)
        ROS_DEBUG_NAMED("grasping_baxter", "Reached Pick up pose for piece.");
      else
        ROS_ERROR_NAMED("grasping_baxter", "Failed to reach Pick up pose for piece.");
      ros::spinOnce();
      moveitcounter++;
    }
    if(moveitcounter>=num_baxtertrys){
      ROS_ERROR_NAMED("grasping_baxter", "FATAL ERROR: Failed to reach Pick up pose for piece. 5x");
    }
  
    sleep(0.5);
    //suck up the piece
    closerightGripper();
    sleep(1.5);  
    ROS_DEBUG_NAMED("grasping_baxter", "Pieces left in game: %i",num_game_pieces_left);  
    
    //move up again 
    //move up following a cartesian path
    waypoints.clear();
    waypoints.push_back(car_target_pose);
    //add more points depending on how many pieces are left
    for(int i =0; i<6-pieces_in_stack+5;i++){
      car_target_pose.position.z += 0.01;
      waypoints.push_back(car_target_pose);
    }
    car_target_pose.position.z += 0.05;
    waypoints.push_back(car_target_pose);
    fraction = group.computeCartesianPath(waypoints,
                                             0.002,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory);
    
    success=false;
    moveitcounter=0;
    group.setStartStateToCurrentState();
    while(!success && moveitcounter <num_baxtertrys){
      my_plan.trajectory_ = trajectory;
      success = group.execute(my_plan);
      sleep(0.1);
      if(success)
        ROS_DEBUG_NAMED("grasping_baxter", "Reached pick_up_pose + some margin with a piece");
      else
        ROS_ERROR_NAMED("grasping_baxter", "Failed to reach pick_up_pose + some margin with a piece");
      ros::spinOnce();
      moveitcounter++;
    }
    if(moveitcounter>=num_baxtertrys){
      ROS_ERROR_NAMED("grasping_baxter", "FATAL ERROR: Failed to reach pick_up_pose + some margin with a piece. 5x");
    }

    sleep(0.1);

  }
  

  void tf_frame_test(){
    target_pose.header.stamp=ros::Time::now();
    target_pose.header.frame_id="/world";
    target_pose=ar_code_pose;
    target_pose.pose.position.z +=0.0;
    target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,0,1.57);
    //move to ar -code
    bool success=false;
    group.setPoseTarget(target_pose);
    success = group.plan(my_plan);
    //move it!!!
    group.move() ;
    sleep(1.0);
    if(success)
      ROS_INFO("WE DID IT!!!!!!!!!!");
    else
      ROS_INFO("Fail");

    // wait
    ros::Duration(10.5).sleep();
    
    //move back
    group.setPoseTarget(pick_up_pose);
    success = group.plan(my_plan);
    //move it!!!
    group.move() ;
    sleep(1.0);
    if(success)
      ROS_INFO("WE DID IT!!!!!!!!!!");
    else
      ROS_INFO("Fail");
  }

  void grasping_baxter_environment_fixed(){
    ROS_DEBUG_NAMED("grasping_baxter","building grasping enviroment fixed");
    //store all collision objects in this vector
    std::vector<moveit_msgs::CollisionObject> all_collision_objects;

    //------kinect--------
    moveit_msgs::CollisionObject co_kinect;
    co_kinect.header.frame_id = group.getPlanningFrame();
    // The id of the object is used to identify it. 
    co_kinect.id = "Kinect";
    shape_msgs::SolidPrimitive cube;
    cube.type = cube.BOX;
    cube.dimensions.resize(3);
    cube.dimensions[0] = 0.2;
    cube.dimensions[1] = 0.3;
    cube.dimensions[2] = 0.2;
    //POSE FOR KINECT
    geometry_msgs::Pose kinect_pose;
    kinect_pose.orientation.w = 1.0;
    kinect_pose.position.x =  0.05;
    kinect_pose.position.y = 0.0;
    kinect_pose.position.z =  0.95;
    //add to collision objects
    co_kinect.primitives.push_back(cube);
    co_kinect.primitive_poses.push_back(kinect_pose);
    co_kinect.operation = co_kinect.ADD;
    all_collision_objects.push_back(co_kinect);

    //------webcam--------
    moveit_msgs::CollisionObject co_webcam;
    co_webcam.header.frame_id = group.getPlanningFrame();
    // The id of the object is used to identify it. 
    co_webcam.id = "webcam";
    shape_msgs::SolidPrimitive cam;
    cam.type = cube.BOX;
    cam.dimensions.resize(3);
    cam.dimensions[0] = 0.1;
    cam.dimensions[1] = 0.1;
    cam.dimensions[2] = 0.05;
    //POSE FOR webcam
    geometry_msgs::Pose webcam_pose;
    webcam_pose.orientation.w = 1.0;
    webcam_pose.position.x =  0.1;
    webcam_pose.position.y = 0.0;
    webcam_pose.position.z =  0.55;
    //add to collision objects
    co_webcam.primitives.push_back(cam);
    co_webcam.primitive_poses.push_back(webcam_pose);
    co_webcam.operation = co_webcam.ADD;
    all_collision_objects.push_back(co_webcam);

    ////------Table model--------
    moveit_msgs::CollisionObject co_table;
    co_table.header.frame_id = group.getPlanningFrame();
    co_table.id = "Table";
    cube.dimensions.resize(3);
    cube.dimensions[0] = 1.0;
    cube.dimensions[1] = 1.2;
    cube.dimensions[2] = 0.74;
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x =  0.75;//0.825;
    table_pose.position.y = 0;
    table_pose.position.z =  -0.51;//-0.575;-0.16
    co_table.primitives.push_back(cube);
    co_table.primitive_poses.push_back(table_pose);
    co_table.operation = co_table.ADD;
    all_collision_objects.push_back(co_table);

    //add all the object to the world
    planning_scene_interface.addCollisionObjects(all_collision_objects);


  }

  //enviroment that changes 
  void grasping_baxter_environment_dynamic(){
    //TODO:make objects rotatable
    ROS_DEBUG_NAMED("grasping_baxter","Updating grasping enviroment");
    // distances
     double gameboard_l=0.405;  //length game board
     double gameboard_w=0.405;  // width game board
     double gameboard_h=0.01; // thickness

    //depends on the ar code
    // calculate p0 pose using x and y ofsset (known)
    double theta=tf::getYaw(ar_code_pose.pose.orientation);
    // calc P0 with ar at origin
    Mat p0 = (Mat_<double>(2,1) <<
                    offset_bc_x,
                    offset_bc_y);
    // rotate around theta
    Mat r = (Mat_<double>(2,2) <<
                    cos(theta), -sin(theta),
                    sin(theta), cos(theta));
    Mat p0r= r*p0;
    //translate to get true pose
    Mat t = (Mat_<double>(2,1)  <<
                    ar_code_pose.pose.position.x,
                    ar_code_pose.pose.position.y);
    p0=p0r+t;
    
    double gameboard_x=p0.at<double>(0,0); //0.825;   // x-pos of gameboard
    double gameboard_y=p0.at<double>(1,0);//gameboard_w;    //
    double gameboard_z= ar_code_pose.pose.position.z+offset_bc_z;; //-0.525;//-0.575;

    double dis_gb_storage_x=0.03;
    //double dis_gb_storage_y=gameboard_y/3;
    double dis_sticks_x=0.05;
    double dis_sticks_y=0.05;
    double radius_cylinder=0.004;
    double heigth_cylinder=0.12;
    double cell_size=0.065;
    double wall_thickness=0.0075;
    double piecebox_comp=0.055;    

    //store all collision objects in this vector
    std::vector<moveit_msgs::CollisionObject> all_collision_objects;
    
    //--------storage object//--------
    moveit_msgs::CollisionObject co_piece_box;
    co_piece_box.header.frame_id = group.getPlanningFrame();
    moveit_msgs::CollisionObject co_table;
    geometry_msgs::Pose table_pose;
    shape_msgs::SolidPrimitive cube;

    // The id of the object is used to identify it. 
    co_piece_box.id = "PieceBox";
    cube.type = cube.BOX;
    cube.dimensions.resize(3);
    cube.dimensions[0] = 3*cell_size;
    cube.dimensions[1] = 0.065;
    cube.dimensions[2] = 0.065;
    //define pose
    geometry_msgs::Pose piece_box_pose;
    piece_box_pose.orientation.w = 1.0;
    piece_box_pose.position.x = gameboard_x+cell_size;
    piece_box_pose.position.y = gameboard_l/2+cell_size+dis_gb_storage_x;
    piece_box_pose.position.z =  -0.14;
    //add to collision objects
    co_piece_box.primitives.push_back(cube);
    co_piece_box.primitive_poses.push_back(piece_box_pose);
    co_piece_box.operation = co_piece_box.ADD;
    all_collision_objects.push_back(co_piece_box);

    // ------ Piece box baxter------//
    //--- wall 1
    moveit_msgs::CollisionObject  co_piece_box_baxter;
    co_piece_box_baxter.header.frame_id = group.getPlanningFrame();
    // The id of the object is used to identify it. 
    
    co_piece_box_baxter.id = "PieceBoxBaxter";
    cube.type = cube.BOX;
    cube.dimensions.resize(3);
    cube.dimensions[0] = 3*cell_size+0.01;
    cube.dimensions[1] = 0.0075;
    cube.dimensions[2] = 0.065;
    // ---- wall 1
    geometry_msgs::Pose piece_box_baxter_pose;
    piece_box_baxter_pose.orientation.w = 1.0;
    piece_box_baxter_pose.position.x = gameboard_x+cell_size/2; //piecebox_comp
    piece_box_baxter_pose.position.y = -gameboard_w/2-dis_gb_storage_x-wall_thickness-piecebox_comp;
    piece_box_baxter_pose.position.z =  -0.14;
    co_piece_box_baxter.primitives.push_back(cube);
    co_piece_box_baxter.primitive_poses.push_back(piece_box_baxter_pose);
    co_piece_box_baxter.operation = co_piece_box_baxter.ADD;
    // ---- wall 2
    piece_box_baxter_pose.orientation.w = 1.0;
    piece_box_baxter_pose.position.x = gameboard_x+cell_size/2;
    piece_box_baxter_pose.position.y = -gameboard_w/2-dis_gb_storage_x;//-gameboard_w/2-dis_gb_storage_x-0.075;
    piece_box_baxter_pose.position.z =  -0.14;
    co_piece_box_baxter.primitives.push_back(cube);
    co_piece_box_baxter.primitive_poses.push_back(piece_box_baxter_pose);
    co_piece_box_baxter.operation = co_piece_box_baxter.ADD;
    all_collision_objects.push_back(co_piece_box_baxter);
  
    // ---- compartment walls -------------------
    moveit_msgs::CollisionObject  co_wall_a;
    co_wall_a.header.frame_id = group.getPlanningFrame();
    // The id of the object is used to identify it. 
    co_wall_a.id = "Wall_A";
    cube.type = cube.BOX;
    cube.dimensions.resize(3);
    cube.dimensions[0] = 0.0065; //height
    cube.dimensions[1] = 0.055; // length
    cube.dimensions[2] = cell_size+0.01; // thickness    

    // ---- wall 1
    geometry_msgs::Pose co_wall_a_pose;
    co_wall_a_pose.orientation.w = 1.0;
    co_wall_a_pose.position.x = gameboard_x-cell_size;
    co_wall_a_pose.position.y = -gameboard_w/2-dis_gb_storage_x-wall_thickness/2- piecebox_comp/2;//-gameboard_w/2-dis_gb_storage_x-0.0075;
    co_wall_a_pose.position.z =  -0.14;
    co_wall_a.primitives.push_back(cube);
    co_wall_a.primitive_poses.push_back(co_wall_a_pose);
    co_wall_a.operation = co_wall_a.ADD;
    // ---- wall 2
    co_wall_a_pose.orientation.w = 1.0;
    co_wall_a_pose.position.x = gameboard_x;
    co_wall_a_pose.position.y = -gameboard_w/2-dis_gb_storage_x-wall_thickness/2- piecebox_comp/2;//-gameboard_w/2-dis_gb_storage_x-0.0075;
    co_wall_a_pose.position.z =  -0.14;
    co_wall_a.primitives.push_back(cube);
    co_wall_a.primitive_poses.push_back(co_wall_a_pose);
    co_wall_a.operation = co_wall_a.ADD;
    // ---- wall 3
    co_wall_a_pose.orientation.w = 1.0;
    co_wall_a_pose.position.x = gameboard_x+2*cell_size;
    co_wall_a_pose.position.y = -gameboard_w/2-dis_gb_storage_x-wall_thickness/2- piecebox_comp/2;//-gameboard_w/2-dis_gb_storage_x-0.0075;
    co_wall_a_pose.position.z =  -0.14;
    co_wall_a.primitives.push_back(cube);
    co_wall_a.primitive_poses.push_back(co_wall_a_pose);
    co_wall_a.operation = co_wall_a.ADD;
    // ---- wall 4
    co_wall_a_pose.orientation.w = 1.0;
    co_wall_a_pose.position.x = gameboard_x+cell_size;
    co_wall_a_pose.position.y = -gameboard_w/2-dis_gb_storage_x-wall_thickness/2- piecebox_comp/2;//-gameboard_w/2-dis_gb_storage_x-0.0075;
    co_wall_a_pose.position.z =  -0.14;
    co_wall_a.primitives.push_back(cube);
    co_wall_a.primitive_poses.push_back(co_wall_a_pose);
    co_wall_a.operation = co_wall_a.ADD;
    all_collision_objects.push_back(co_wall_a);

    //--- gameboard model
    cube.dimensions.resize(3);
    cube.dimensions[0] = gameboard_l;
    cube.dimensions[1] = gameboard_w;
    cube.dimensions[2] = gameboard_h;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = gameboard_x; 
    table_pose.position.y = gameboard_y;
    table_pose.position.z = gameboard_z;
    co_table.primitives.push_back(cube);
    co_table.primitive_poses.push_back(table_pose);
    co_table.operation = co_table.ADD;
    
    //add all the object to the world
    planning_scene_interface.addCollisionObjects(all_collision_objects);


  }


  

  //grasping baxter command function
  void grasping_baxter_start_command(const grasping_baxter::grasping_baxter_game_masterGoalConstPtr &goal)
  {
    ROS_DEBUG_NAMED("grasping_baxter", "%s: start received", action_name_.c_str());
    //stop updating the ar-pose when we move around
    doarupdate=false;

    //is it a special request?
    if(goal->move==100 && goal->pieces==100){
      //performe special move
      move_to_ground_pose();
      result_grasping_baxter.done_grasping = 1;
      ROS_DEBUG_NAMED("grasping_baxter", "%s: Done", action_name_.c_str());
      // set the action state to succeeded
      as_grasping_baxter.setSucceeded(result_grasping_baxter);
      return;
    }
    //is it a special request?
    if(goal->move==200 && goal->pieces==200){
      //performe special move
      check_reach();
      result_grasping_baxter.done_grasping = 1;
      ROS_DEBUG_NAMED("grasping_baxter", "%s: Done", action_name_.c_str());
      // set the action state to succeeded
      as_grasping_baxter.setSucceeded(result_grasping_baxter);
      return;
    }

    //where do we want to go?
    target_field =goal->move;
    num_game_pieces_left =goal->pieces;
    //use RRT 
    group.setPlannerId("RRTConnectkConfigDefault");
    ros::Duration(0.5).sleep();

    //adjust pick up pose depending on which tower we are picking from
    // rotate around theta    
    double theta=tf::getYaw(ar_code_pose.pose.orientation);
    Mat r = (Mat_<double>(2,2) <<
                    cos(theta), -sin(theta),
                    sin(theta), cos(theta));
    
    //translate to get true pose
    Mat t = (Mat_<double>(2,1)  <<
                    0.0625,
                    0.0);
    
    t = r*t;
    offset_ss_x =  t.at<double>(0,0);
    offset_ss_y =  t.at<double>(1,0);

    //set the current stack
    if(num_game_pieces_left<19){
      current_stack=0;
    }
    if(num_game_pieces_left<13){
      current_stack=1;
    }
    if(num_game_pieces_left<7){
      current_stack=2;
    }

    //account for the current stack
    pick_up_pose.pose.position.x+=offset_ss_x*current_stack;
    pick_up_pose.pose.position.y+=offset_ss_y*current_stack;
    //tolerances
    group.setGoalOrientationTolerance(0.0005);
    group.setGoalPositionTolerance(0.0005);
    group.setGoalTolerance(0.0005);
    group.setNumPlanningAttempts(2);
    group.setPlanningTime(5);
    group.setStartStateToCurrentState();
    
    //do a thing
    //target_field=0;
    if(num_game_pieces_left==18)
    {
      pick_up_firs_piece();
    }
    place_piece();
    //tf_frame_test();
    result_grasping_baxter.done_grasping = 1;
    ROS_DEBUG_NAMED("grasping_baxter", "%s: Done", action_name_.c_str());
    // set the action state to succeeded
    as_grasping_baxter.setSucceeded(result_grasping_baxter);
    //start doing position updates again
    doarupdate=true;
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasping_baxter_game_master");
  ROS_INFO("Start grasping node");
  //start action server
  grasping_baxter_boss gbb("grasping_baxter_game_master");

  while(ros::ok()){
  	gbb.publish_poses();
  	ros::spinOnce();
  	ros::Duration(0.5).sleep(); 
  }
}
