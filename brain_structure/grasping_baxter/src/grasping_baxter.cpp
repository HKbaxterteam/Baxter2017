//************************************************
//**********Baxter 2017 Tic-Tac-Toe***************
//*******Nadine Drollinger & Michael Welle********
//************************************************
//*******Grasping node - grasping_baxter *********
//************************************************

//************************************************
//Description: gets the ar-tag postioin and the 
// field number. Calculates pick and place poses 
// and does it.
//************************************************

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <grasping_baxter/grasping_baxter_game_masterAction.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include  <tf/transform_datatypes.h>

#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/DigitalIOState.h>

#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;


class grasping_baxter_boss
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<grasping_baxter::grasping_baxter_game_masterAction> as_grasping_baxter; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  grasping_baxter::grasping_baxter_game_masterFeedback feedback_grasping_baxter; // create messages that are used to published feedback
  grasping_baxter::grasping_baxter_game_masterResult result_grasping_baxter;    // create messages that are used to published result

public:

	bool grasping_baxter_start_flag;
  bool debug_flag;
  moveit::planning_interface::MoveGroup group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroup::Plan my_plan;
  geometry_msgs::PoseStamped target_posehope;
  ros::Publisher ar_pub;
  ros::Publisher target_pub;
  ros::Publisher pp_pub;
  ros::Publisher rightGripperPub;
  ros::Subscriber ar_pose_sub;

  geometry_msgs::PoseStamped ar_code_pose;
  geometry_msgs::PoseStamped target_pose;
  geometry_msgs::PoseStamped p0_pose;
  geometry_msgs::PoseStamped pick_up_pose;
  int target_field;
  int gripperSeq;

  //monitor game states
  int num_game_pieces_left;

  //ofset yaw for ar codes
  double offset_ar_tag_yaw;

  // offset for from ar to 
  double offset_p0_pose_x;
  double offset_p0_pose_y;
  double offset_p0_pose_z;
  
  // offset for piece-box storage
  double offset_pick_up_pose_x;
  double offset_pick_up_pose_y;
  double offset_pick_up_pose_z;

  //x a y offset for field to field
  double offset_ff_x;
  double offset_ff_y;
  // x a y offset stack to stack
  double offset_ss_x;
  double offset_ss_y;

  //ar-tag var
  std::vector<geometry_msgs::PoseStamped> ar_tag_pose_vector;
  int art_vec_count;
  int art_vec_position;
  //offset_ar_tag_yaw(-12.3*3.1415926/180)
  //constructor
  grasping_baxter_boss(std::string name) :
    as_grasping_baxter(nh_, name, boost::bind(&grasping_baxter_boss::grasping_baxter_start_command, this, _1), false),
    action_name_(name), grasping_baxter_start_flag(false),group("right_arm"),offset_p0_pose_x(-0.115),offset_p0_pose_y(+0.16),
    offset_p0_pose_z(0.05),offset_pick_up_pose_x(-0.27),offset_pick_up_pose_y(-0.26),offset_pick_up_pose_z(0.09),art_vec_count(0),
    art_vec_position(0),ar_tag_pose_vector(10),debug_flag(false),num_game_pieces_left(18),offset_ar_tag_yaw(0*3.1415926/180)
  {
    // start action server
    as_grasping_baxter.start();

    //poses pub and subscribers
    ar_pub = nh_.advertise<geometry_msgs::PoseStamped>("/poses/ar_code", 1);
    target_pub = nh_.advertise<geometry_msgs::PoseStamped>("/poses/target", 1);
    pp_pub = nh_.advertise<geometry_msgs::PoseStamped>("/poses/pick_place", 1);
    ar_pose_sub = nh_.subscribe("/TTTgame/ar_tag/ar_pose", 1, &grasping_baxter_boss::ar_code_pose_callback, this);
    rightGripperPub = nh_.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command",10, true);

    //calibrate gripper
    calibraterightGripper();

    grasping_baxter_environment();
  }

  //deconstructor
  ~grasping_baxter_boss(void)
  {
    

  }

  //calibrate right Gripper
  bool calibraterightGripper()
  {
    ROS_INFO("Calibrate right arm gripper");
    
    baxter_core_msgs::EndEffectorCommand command;
    command.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
    command.id = 65537;
    command.sender = "grasping_baxter";
    command.sequence = gripperSeq++;   
    rightGripperPub.publish(command);        
    ros::Duration(0.2).sleep();
    return true;
  }

  //close right Gripper
  bool closerightGripper()
  {
    ROS_INFO("Closing right arm gripper");
    
    baxter_core_msgs::EndEffectorCommand command;
    command.command = baxter_core_msgs::EndEffectorCommand::CMD_GRIP;
    command.id = 65537;
    command.sequence = gripperSeq++;
    command.sender = "grasping_baxter";
    rightGripperPub.publish(command);    
    ros::Duration(0.2).sleep();
    return true;
  }

  //open right gripper
  bool openrightGripper()
  {
    ROS_INFO("Opening right arm gripper");

    baxter_core_msgs::EndEffectorCommand command;
    command.command = baxter_core_msgs::EndEffectorCommand::CMD_RELEASE;
    command.id = 65537;
    command.sequence = gripperSeq++;
    command.sender = "grasping_baxter";
    rightGripperPub.publish(command);
    ros::Duration(0.2).sleep();
    return true;
  }

  //Calback from ar-tag, takes the avarage of 100 poses (in sliding avarage mode) 
  // and calculates pickup pose and p0 pose debending on it.
  void ar_code_pose_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
  {
    //cout << "callbocko" << endl;
    geometry_msgs::PoseStamped ar_code_l_pose;
    geometry_msgs::PoseStamped ar_code_r_pose;
    double theta;
    //check if there are 2 poses
    if(msg->markers.size()!=2){
      ROS_INFO("Not 2 Markers");
      return;
    }

    
        //get left marker
      if(msg->markers[0].pose.pose.position.y > msg->markers[1].pose.pose.position.y )
        ar_code_l_pose=msg->markers[0].pose;

      if(msg->markers[1].pose.pose.position.y > msg->markers[0].pose.pose.position.y)
        ar_code_l_pose=msg->markers[1].pose;

      //get right marker
      if(msg->markers[0].pose.pose.position.y < msg->markers[1].pose.pose.position.y )
        ar_code_r_pose=msg->markers[0].pose;

      if(msg->markers[1].pose.pose.position.y < msg->markers[0].pose.pose.position.y)
        ar_code_r_pose=msg->markers[1].pose;

      
      //comine the two ar_code poses to one
      ar_code_pose.header.frame_id="/world";
      ar_code_pose.header.stamp = ros::Time::now();;
      // we the abs diff of the markers and half them
      if(ar_code_l_pose.pose.position.x>ar_code_r_pose.pose.position.x)
        ar_code_pose.pose.position.x = ar_code_l_pose.pose.position.x - (ar_code_l_pose.pose.position.x-ar_code_r_pose.pose.position.x)/2;

      if(ar_code_l_pose.pose.position.x<=ar_code_r_pose.pose.position.x)
        ar_code_pose.pose.position.x = ar_code_l_pose.pose.position.x + (ar_code_r_pose.pose.position.x-ar_code_l_pose.pose.position.x)/2;

      
      ar_code_pose.pose.position.y = ar_code_r_pose.pose.position.y+(ar_code_l_pose.pose.position.y-ar_code_r_pose.pose.position.y)/2;
      
      ar_code_pose.pose.position.z = (ar_code_l_pose.pose.position.z +ar_code_r_pose.pose.position.z)/2;



      //cout << "ar code z: " << ar_code_pose.pose.position.z << " l : " << ar_code_l_pose.pose.position.z << " r : " << ar_code_r_pose.pose.position.z << endl;
      
      // get yaw
      double theta_l = tf::getYaw(ar_code_l_pose.pose.orientation)+offset_ar_tag_yaw;
      double theta_r = tf::getYaw(ar_code_r_pose.pose.orientation)+offset_ar_tag_yaw;

      theta = (theta_l+theta_r)/2;
      ar_code_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);


       //sliding avarage calculation vor ar-tag pose
      //cout << "vector pos: " << art_vec_position << "****************************" << endl;
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



    // calculate p0 pose using x and y ofsset (known)
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
    //set oriantion
    //NOTE: not sure why baxter need this oriantation to grasp from the top ...
    p0_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,0,1.57);

    //calculate offsets to other P (p1-58)
    t = (Mat_<double>(2,1)  <<
                    -0.065,
                    -0.065);
    t = r*t;

    offset_ff_x =  t.at<double>(0,0);
    offset_ff_y =  t.at<double>(1,0);


    //calculate pickup pose pose dependig on how many pieces are left

    pick_up_pose.header.stamp=ros::Time::now();
    pick_up_pose.header.frame_id="/world";

    // calc Ppu with ar at origin
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
    //NOTE: not sure why baxter need this oriantation to grasp from the top ...
    pick_up_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,0,1.57);

    //addjust pick up pose depending on on which tower we are emptieng
    //calc tower offsets
    t = (Mat_<double>(2,1)  <<
                    0.065,
                    0.0);
    t = r*t;

    offset_ss_x =  t.at<double>(0,0);
    offset_ss_y =  t.at<double>(1,0);
    

    //second tower
    if(num_game_pieces_left<13){
      pick_up_pose.pose.position.x=pick_up_pose.pose.position.x+offset_ss_x;
      pick_up_pose.pose.position.y=pick_up_pose.pose.position.y+offset_ss_y;
    }
    //3 tower
    if(num_game_pieces_left<7){
      pick_up_pose.pose.position.x=pick_up_pose.pose.position.x+2*offset_ss_x;
      pick_up_pose.pose.position.y=pick_up_pose.pose.position.y+2*offset_ss_y;
    }
   
  }
  
  void publish_goalpose(){
  	ar_pub.publish(ar_code_pose);    
    target_pub.publish(target_pose);
    pp_pub.publish(pick_up_pose);
  }

//function to pick up a piece and place it where the Ai wants it
  void place_piece(){

   

   //define target pose
    target_pose.header.stamp=ros::Time::now();
    target_pose.header.frame_id="/world";
    target_pose=pick_up_pose;

    //move to pick up pose    
    bool success=false;
    group.setPoseTarget(target_pose);
    success = group.plan(my_plan);
    //move it!!!
    //group.move() ;
    //try execute

    while(!group.execute(my_plan)){
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ROS_INFO("Wait for exodus");
    }
    sleep(0.1);
    if(success)
      ROS_INFO("WE DID IT!!!!!!!!!!");
    else
      ROS_INFO("Fail");

    //move down depending on how many pieces are left
    int pieces_in_stack=num_game_pieces_left % 6;
    if(num_game_pieces_left % 6==0)
      pieces_in_stack=6;
    sleep(0.1);

      
    // move down depeding on pieces ins tack
    target_pose.pose.position.z = target_pose.pose.position.z - (6-pieces_in_stack)*0.01;
    
    success=false;
    group.setPoseTarget(target_pose);
    success = group.plan(my_plan);
    //move it!!!
    group.move() ;
    sleep(0.1);
    if(success)
      ROS_INFO("WE DID IT!!!!!!!!!!");
    else
      ROS_INFO("Fail");

    //suck up the piece
    closerightGripper();
    ros::Duration(0.5).sleep();  

    //decrese num of game pieces
    num_game_pieces_left--;  

    //move up again?
    target_pose.pose.position.z = pick_up_pose.pose.position.z;

    success=false;
    group.setPoseTarget(target_pose);
    success = group.plan(my_plan);
    //move it!!!
    group.move() ;
    sleep(0.1);
    if(success)
      ROS_INFO("WE DID IT!!!!!!!!!!");
    else
      ROS_INFO("Fail");

   
    //move to place pos with the pick up z  
    //calculate the field possition
    int row=target_field/6;
    int col=target_field%6;

    target_pose.pose.position.x=p0_pose.pose.position.x+row*offset_ff_x;
    target_pose.pose.position.y=p0_pose.pose.position.y+col*offset_ff_y;
    std::cout << "reaching field n" << target_field << std::endl;
    success=false;
    group.setPoseTarget(target_pose);
    success = group.plan(my_plan);
    //move it!!!
    //group.move() ;
    while(!group.execute(my_plan)){
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ROS_INFO("Wait for exodus 2");
    }
    sleep(1.0);
    if(success)
      ROS_INFO("WE DID IT!!!!!!!!!!");
    else
      ROS_INFO("Fail");
    sleep(0.1);

    // move down to p0 z pose
    target_pose.pose.position.z = p0_pose.pose.position.z;
    success=false;
    group.setPoseTarget(target_pose);
    success = group.plan(my_plan);
    //move it!!!
    group.move() ;
    sleep(1.0);
    if(success)
      ROS_INFO("WE DID IT!!!!!!!!!!");
    else
      ROS_INFO("Fail");

    //release the piece
    openrightGripper();

    //move to pick up pose   
    target_pose=pick_up_pose; 
    success=false;
    group.setPoseTarget(target_pose);
    success = group.plan(my_plan);
    //move it!!!
    //group.move() ;
    while(!group.execute(my_plan)){
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ROS_INFO("Wait for exodus3");
    }
    sleep(1.0);
    if(success)
      ROS_INFO("WE DID IT!!!!!!!!!!");
    else
      ROS_INFO("Fail");
    sleep(0.1);
 
  }

// picks up a pice and delivers it to evry position on the board
  void picking_test(){

    //simulate callback
    //ar_code_pose_callback();

    target_pose.header.stamp=ros::Time::now();
    target_pose.header.frame_id="/world";
    target_pose=p0_pose;

    int i=0;
    double offset_x=-0.065;
    double offset_y=-0.065;

    while(i <= 35) {
      //move to pick up pose
      bool success=false;
      group.setPoseTarget(pick_up_pose);
      success = group.plan(my_plan);
      //move it!!!
      group.move() ;
      sleep(1.0);
//          ros::spinOnce();
      if(success)
        ROS_INFO("WE DID IT!!!!!!!!!!");
      else
        ROS_INFO("Fail");

    //suck up the piece
    closerightGripper();
    ros::Duration(0.5).sleep();
    
    //quick wait
    ros::Duration(0.5).sleep();
    
    //move to place pose
    success=false;
    int row=i/6;
    int col=i%6;

    //TODO: fix calculation
    target_pose.pose.position.x=p0_pose.pose.position.x+row*offset_x;
    target_pose.pose.position.y=p0_pose.pose.position.y+col*offset_y;
    target_pose.pose.position.z=pick_up_pose.pose.position.z;

    std::cout << "reaching field n" << i << std::endl;
    group.setPoseTarget(target_pose);
    success = group.plan(my_plan);
    //move it!!!
    group.move() ;
    sleep(1.0);
    if(success)
      ROS_INFO("WE DID IT!!!!!!!!!!");
    else
      ROS_INFO("Fail");

    //stop sucking
    openrightGripper();
    
    i++;

   }

  }


   void tf_frame_test(){

    //simulate callback
    //ar_code_pose_callback();

    target_pose.header.stamp=ros::Time::now();
    target_pose.header.frame_id="/world";
    target_pose=ar_code_pose;


    //NOTE: not sure why baxter need this oriantation to grasp from the top ...
    target_pose.pose.position.z +=0.0;
    target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,0,1.57);

   //move to ar -code
      bool success=false;
      group.setPoseTarget(target_pose);
      success = group.plan(my_plan);
      //move it!!!
      group.move() ;
      sleep(1.0);
//          ros::spinOnce();
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

//define the starting envirorment
  void grasping_baxter_environment(){

  	// distances
  	 double gameboard_l=0.60;	//length game board
  	 double gameboard_w=0.60;	// width game board
  	 double gameboard_h=0.01;	// thickness

  	 double gameboard_x=0.61; //0.825;		// x-pos of gameboard
  	 double gameboard_y=0;//gameboard_w;		//
  	 double gameboard_z= -0.155; //-0.525;//-0.575;
  	 double dis_gb_storage_x=0.003;
  	 //double dis_gb_storage_y=gameboard_y/3;
  	 double dis_sticks_x=0.05;
  	 double dis_sticks_y=0.05;
	 double radius_cylinder=0.004;
	 double heigth_cylinder=0.12;
	 double cell_size=0.065;

  	
    if(debug_flag){
      ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
      ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
    }
  
    //tolerances
    group.setGoalOrientationTolerance(0.005);
    group.setGoalPositionTolerance(0.005);
    group.setGoalTolerance(0.005);
    group.setNumPlanningAttempts(2);
    group.setPlanningTime(20);
    group.setStartStateToCurrentState();

  	//store all collision object in this vector
  	std::vector<moveit_msgs::CollisionObject> all_collision_objects;

  	//kinect object
    moveit_msgs::CollisionObject co_kinect;
	  co_kinect.header.frame_id = group.getPlanningFrame();

  	// The id of the object is used to identify it. 
	  co_kinect.id = "Kinect";

  	// Define a box to add to the world. 
  	shape_msgs::SolidPrimitive cube;
  	cube.type = cube.BOX;
  	cube.dimensions.resize(3);
  	cube.dimensions[0] = 0.2;
  	cube.dimensions[1] = 0.3;
  	cube.dimensions[2] = 0.2;

  	// A pose for the box (specified relative to frame_id) 
  	geometry_msgs::Pose kinect_pose;
  	kinect_pose.orientation.w = 1.0;
  	kinect_pose.position.x =  0.05;
  	kinect_pose.position.y = 0.0;
  	kinect_pose.position.z =  0.95;

  	co_kinect.primitives.push_back(cube);
  	co_kinect.primitive_poses.push_back(kinect_pose);
  	co_kinect.operation = co_kinect.ADD;

  	all_collision_objects.push_back(co_kinect);

  	//------------------------------storage object
    moveit_msgs::CollisionObject co_piece_box;
	co_piece_box.header.frame_id = group.getPlanningFrame();

  	// The id of the object is used to identify it. 
	  co_piece_box.id = "PieceBox";

  	// Define a box to add to the world. 
  	//shape_msgs::SolidPrimitive cube;
  	cube.type = cube.BOX;
  	cube.dimensions.resize(3);
  	cube.dimensions[0] = 3*cell_size;
  	cube.dimensions[1] = 0.065;
  	cube.dimensions[2] = 0.065;

  	// A pose for the box (specified relative to frame_id) 
  	geometry_msgs::Pose piece_box_pose;
    piece_box_pose.orientation.w = 1.0;
  	piece_box_pose.position.x = gameboard_x+cell_size;
  	piece_box_pose.position.y = gameboard_l/2+cell_size+dis_gb_storage_x;
  	piece_box_pose.position.z =  -0.14;

  	 co_piece_box.primitives.push_back(cube);
  	 co_piece_box.primitive_poses.push_back(piece_box_pose);
  	 co_piece_box.operation = co_piece_box.ADD;

	all_collision_objects.push_back(co_piece_box);

 

  	/*++++++++++Table model ##############*/
  	moveit_msgs::CollisionObject co_table;
  	co_table.header.frame_id = group.getPlanningFrame();
  	co_table.id = "Table";

  	cube.dimensions.resize(3);
  	cube.dimensions[0] = 0.65;
  	cube.dimensions[1] = 0.65;
  	cube.dimensions[2] = 0.85;

  	// A pose for the box (specified relative to frame_id) 
  	geometry_msgs::Pose table_pose;

    table_pose.orientation.w = 1.0;
  	table_pose.position.x =  0.61;//0.825;
  	table_pose.position.y = 0;
  	table_pose.position.z =  -0.525;//-0.575;

  	co_table.primitives.push_back(cube);
  	co_table.primitive_poses.push_back(table_pose);

  	co_table.operation = co_table.ADD;

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



  	all_collision_objects.push_back(co_table);

  //++++++++++Cylinder model ##############
  	
  	moveit_msgs::CollisionObject co_cylinder;
  	co_cylinder.header.frame_id = group.getPlanningFrame();
  	co_cylinder.id = "Cylinder";

  	shape_msgs::SolidPrimitive cylinder;
  	cylinder.type = cube.CYLINDER;
  	cylinder.dimensions.resize(2);
  	cylinder.dimensions[0] = heigth_cylinder; //heigth
  	cylinder.dimensions[1] = radius_cylinder; //radius

  	geometry_msgs::Pose cylinder_pose;

  	//----------- position Cylinder 1
  	cylinder_pose.orientation.w = 1.0;
  	cylinder_pose.position.x = gameboard_x+3*cell_size;// 0.41;//0.825;
  	cylinder_pose.position.y = -gameboard_w/2-dis_gb_storage_x-dis_sticks_y;// 0.415;
  	cylinder_pose.position.z =  gameboard_z;//-0.575;

  	co_cylinder.primitives.push_back(cylinder);
  	co_cylinder.primitive_poses.push_back(cylinder_pose);
  	co_cylinder.operation = co_cylinder.ADD;

  	//----------- position Cylinder 2
  	cylinder_pose.orientation.w = 1.0;
  	cylinder_pose.position.x = gameboard_x+3*cell_size;//0.825;
  	cylinder_pose.position.y = -gameboard_w/2-dis_gb_storage_x-2*dis_sticks_y;
  	cylinder_pose.position.z =  gameboard_z;//-0.575;

  	co_cylinder.primitives.push_back(cylinder);
  	co_cylinder.primitive_poses.push_back(cylinder_pose);
  	co_cylinder.operation = co_cylinder.ADD;

  	 //----------- position Cylinder 3
  	cylinder_pose.orientation.w = 1.0;
  	cylinder_pose.position.x = gameboard_x+2*cell_size;// 0.41;//0.825;
  	cylinder_pose.position.y =  -gameboard_w/2-dis_gb_storage_x-dis_sticks_y;
  	cylinder_pose.position.z =  gameboard_z;//-0.575;

  	co_cylinder.primitives.push_back(cylinder);
  	co_cylinder.primitive_poses.push_back(cylinder_pose);
  	co_cylinder.operation = co_cylinder.ADD;

  	 //----------- position Cylinder 4
  	cylinder_pose.orientation.w = 1.0;
  	cylinder_pose.position.x = gameboard_x+2*cell_size;// 0.41;//0.825;
  	cylinder_pose.position.y =  -gameboard_w/2-dis_gb_storage_x-2*dis_sticks_y;
  	cylinder_pose.position.z =  gameboard_z;//-0.575;


  	co_cylinder.primitives.push_back(cylinder);
  	co_cylinder.primitive_poses.push_back(cylinder_pose);
  	co_cylinder.operation = co_cylinder.ADD;

  	 //----------- positionCylinder 5
  	cylinder_pose.orientation.w = 1.0;
  	cylinder_pose.position.x = gameboard_x+cell_size;// 0.41;//0.825;
  	cylinder_pose.position.y =  -gameboard_w/2-dis_gb_storage_x-dis_sticks_y;
  	cylinder_pose.position.z =  gameboard_z;//-0.575;

  	co_cylinder.primitives.push_back(cylinder);
  	co_cylinder.primitive_poses.push_back(cylinder_pose);
  	co_cylinder.operation = co_cylinder.ADD;

  	//----------- position Cylinder 6
  	cylinder_pose.orientation.w = 1.0;
  	cylinder_pose.position.x = gameboard_x+cell_size;// 0.41;//0.825;
  	cylinder_pose.position.y =  -gameboard_w/2-dis_gb_storage_x-2*dis_sticks_y;
  	cylinder_pose.position.z =  gameboard_z;//-0.575;

  	co_cylinder.primitives.push_back(cylinder);
  	co_cylinder.primitive_poses.push_back(cylinder_pose);
  	co_cylinder.operation = co_cylinder.ADD;

  	//----------- position Cylinder 7
  	cylinder_pose.orientation.w = 1.0;
  	cylinder_pose.position.x = gameboard_x;// 0.41;//0.825;
  	cylinder_pose.position.y =  -gameboard_w/2-dis_gb_storage_x-dis_sticks_y;
  	cylinder_pose.position.z =  gameboard_z;//-0.575;

  	co_cylinder.primitives.push_back(cylinder);
  	co_cylinder.primitive_poses.push_back(cylinder_pose);
  	co_cylinder.operation = co_cylinder.ADD;

  	//----------- position Cylinder 8
  	cylinder_pose.orientation.w = 1.0;
  	cylinder_pose.position.x = gameboard_x;// 0.41;//0.825;
  	cylinder_pose.position.y =  -gameboard_w/2-dis_gb_storage_x-2*dis_sticks_y;
  	cylinder_pose.position.z =  gameboard_z;//-0.575;

  	co_cylinder.primitives.push_back(cylinder);
  	co_cylinder.primitive_poses.push_back(cylinder_pose);
  	co_cylinder.operation = co_cylinder.ADD;

  	all_collision_objects.push_back(co_cylinder); 
  	





  	ROS_INFO("Add an objects into the world");
	  planning_scene_interface.addCollisionObjects(all_collision_objects);
	
  }

  void grasping_baxter_start_command(const grasping_baxter::grasping_baxter_game_masterGoalConstPtr &goal)
  {
  	
    //make the enviroment
    //grasping_baxter_environment();
    
    
    //wher do we want to go?
    target_field =goal->move;
    //use RRT 
    group.setPlannerId("RRTConnectkConfigDefault");
    ros::Duration(0.5).sleep();
    
    //do a thing
    //place_piece();
    //picking_test();
    tf_frame_test();
    
    //feedback
    feedback_grasping_baxter.progress=20; // progress in %

    // publish info to the console for the user
    ROS_INFO("%s: start received", action_name_.c_str());
	  //fedback that everything is ok
    as_grasping_baxter.publishFeedback(feedback_grasping_baxter);
    bool success=true;

    if(success)
    {
    	ROS_INFO("executing move ");
      result_grasping_baxter.done_grasping = 1;
      ROS_INFO("%s: Done", action_name_.c_str());
      // set the action state to succeeded
      as_grasping_baxter.setSucceeded(result_grasping_baxter);
      grasping_baxter_start_flag=true;
    }
  }



};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasping_baxter_game_master");
  ROS_INFO("Start grasping node");
  //start action server
  grasping_baxter_boss gbb("grasping_baxter_game_master");

  while(ros::ok()){
  	gbb.publish_goalpose();
  	//ROS_INFO("publishing");
  	ros::spinOnce();
  	ros::Duration(0.5).sleep(); 
  }

  //ros::spin();

}
