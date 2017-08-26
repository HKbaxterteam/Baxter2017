
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
    ros::Publisher rightGripperPub;

  	geometry_msgs::PoseStamped ar_code_pose;
    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseStamped p0_pose;
    geometry_msgs::PoseStamped pick_up_pose;
  int target_field;
  int gripperSeq;


// offset for from ar to 
  double offset_p0_pose_x;
  double offset_p0_pose_y;
  double offset_p0_pose_z;
  double offset_p0_orientation_x;
  double offset_p0_orientation_y;
  double offset_p0_orientation_z;
  double offset_p0_orientation_w;


// offset for piece-box storage
  double offset_pick_up_pose_x;
  double offset_pick_up_pose_y;
  double offset_pick_up_pose_z;
  double offset_pick_up_orientation_x;
  double offset_pick_up_orientation_y;
  double offset_pick_up_orientation_z;
  double offset_pick_up_orientation_w;




  grasping_baxter_boss(std::string name) :
    as_grasping_baxter(nh_, name, boost::bind(&grasping_baxter_boss::grasping_baxter_start_command, this, _1), false),
    action_name_(name), grasping_baxter_start_flag(false),group("right_arm"),offset_p0_pose_x(-0.12),offset_p0_pose_y(+0.185),
    offset_p0_pose_z(0),offset_p0_orientation_x(0),offset_p0_orientation_y(0),offset_p0_orientation_z(0),
    offset_p0_orientation_w(0),offset_pick_up_pose_x(-0.18),offset_pick_up_pose_y(-0.285),offset_pick_up_pose_z(0.07),
    offset_pick_up_orientation_x(0),offset_pick_up_orientation_y(0),offset_pick_up_orientation_z(0),offset_pick_up_orientation_w(0),
    debug_flag(true)
  {
    as_grasping_baxter.start();
    ar_pub = nh_.advertise<geometry_msgs::PoseStamped>("/poses/ar_code", 1);
    target_pub = nh_.advertise<geometry_msgs::PoseStamped>("/poses/target", 1);

    rightGripperPub = nh_.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command",10, true);




    //callback will do that later:
    ar_code_pose.header.frame_id="/world";
    ar_code_pose.header.stamp = ros::Time::now();;
    ar_code_pose.pose.position.x=0.915;
    ar_code_pose.pose.position.y=-0.15;
    ar_code_pose.pose.position.z=-0.145608429006;

    ar_code_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,0,1.57);
    

 

  }

  ~grasping_baxter_boss(void)
  {
    

  }

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

 void ar_code_pose_callback()
 {

  //THis will be the callback from the kinect


    //calculate p0 pose
      p0_pose.header.stamp=ros::Time::now();
      p0_pose.header.frame_id="/world";

      p0_pose.pose.position.x=ar_code_pose.pose.position.x+offset_p0_pose_x;
      p0_pose.pose.position.y=ar_code_pose.pose.position.y+offset_p0_pose_y;
      p0_pose.pose.position.z=ar_code_pose.pose.position.z+offset_p0_pose_z;
      p0_pose.pose.orientation.x=ar_code_pose.pose.orientation.x+offset_p0_orientation_x;
      p0_pose.pose.orientation.y=ar_code_pose.pose.orientation.y+offset_p0_orientation_y;
      p0_pose.pose.orientation.z=ar_code_pose.pose.orientation.z+offset_p0_orientation_z;
      p0_pose.pose.orientation.w=ar_code_pose.pose.orientation.w+offset_p0_orientation_w;


    //calculate pickup pose pose
      pick_up_pose.header.stamp=ros::Time::now();
      pick_up_pose.header.frame_id="/world";

      pick_up_pose.pose.position.x=ar_code_pose.pose.position.x+offset_pick_up_pose_x;
      pick_up_pose.pose.position.y=ar_code_pose.pose.position.y+offset_pick_up_pose_y;
      pick_up_pose.pose.position.z=ar_code_pose.pose.position.z+offset_pick_up_pose_z;
      pick_up_pose.pose.orientation.x=ar_code_pose.pose.orientation.x+offset_pick_up_orientation_x;
      pick_up_pose.pose.orientation.y=ar_code_pose.pose.orientation.y+offset_pick_up_orientation_y;
      pick_up_pose.pose.orientation.z=ar_code_pose.pose.orientation.z+offset_pick_up_orientation_z;
      pick_up_pose.pose.orientation.w=ar_code_pose.pose.orientation.w+offset_pick_up_orientation_w;

  }
  
  void publish_goalpose(){
  	ar_pub.publish(pick_up_pose);    
    target_pub.publish(target_pose);
  }

//function to pick up a piece and place it where the Ai wants it
  void place_piece(){

    //simulate ar callback  
    ar_code_pose_callback();

    //define target pose
    target_pose.header.stamp=ros::Time::now();
    target_pose.header.frame_id="/world";
    target_pose=pick_up_pose;

    //move to pick up pose    
    bool success=false;
    group.setPoseTarget(target_pose);
    success = group.plan(my_plan);
    //move it!!!
    group.move() ;
    sleep(5.0);
    if(success)
      ROS_INFO("WE DID IT!!!!!!!!!!");
    else
      ROS_INFO("Fail");
  
    //suck up the piece
    closerightGripper();
    ros::Duration(0.5).sleep();
    

    //move to place pos
    success=false;

    //TODO: calculate possition invariant to angelar errors
    double offset_x=-0.065;
    double offset_y=-0.065;

    int row=target_field/7;
    int col=target_field%7;

    target_pose.pose.position.x=p0_pose.pose.position.x+row*offset_x;
    target_pose.pose.position.y=p0_pose.pose.position.y+col*offset_y;

    std::cout << "reaching field n" << target_field << std::endl;;

    group.setPoseTarget(target_pose);
    success = group.plan(my_plan);
    //move it!!!
    group.move() ;
    sleep(1.0);
    if(success)
      ROS_INFO("WE DID IT!!!!!!!!!!");
    else
      ROS_INFO("Fail");

    openrightGripper();

    //TODO:move away to prepare pos

  }

// picks up a pice and delivers it to evry position on the board
  void picking_test(){

    //simulate callback
    ar_code_pose_callback();

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

//define the starting envirorment
  void grasping_baxter_environment(){

  	// distances
  	 double gameboard_l=0.60;	//length game board
  	 double gameboard_w=0.60;	// width game board
  	 double gameboard_h=0.01;	// thickness

  	 double gameboard_x=0.61; //0.825;		// x-pos of gameboard
  	 double gameboard_y=0;//gameboard_w;		//
  	 double gameboard_z= -0.14; //-0.525;//-0.575;
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
    grasping_baxter_environment();
    
    bool success=false;
    //wher do we want to go?
    target_field =goal->move;
    //use RRT 
    group.setPlannerId("RRTConnectkConfigDefault");
    ros::Duration(0.5).sleep();
    //calibrate gripper
    calibraterightGripper();
    
    //do a thing
    //place_piece();
    picking_test();
    
    //feedback
    feedback_grasping_baxter.progress=20; // progress in %

    // publish info to the console for the user
    ROS_INFO("%s: start received", action_name_.c_str());
	  //fedback that everything is ok
    as_grasping_baxter.publishFeedback(feedback_grasping_baxter);
    

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

  ros::spin();

}
