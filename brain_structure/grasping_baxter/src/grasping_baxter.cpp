
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


/*------------- old names
   reachability test
  double offset_qr_pose_pick_up_x;
  double offset_qr_pose_pick_up_y;
  double offset_qr_pose_pick_up_z;
  double offset_qr_orientation_pick_up_x;
  double offset_qr_orientation_pick_up_y;
  double offset_qr_orientation_pick_up_z;
  double offset_qr_orientation_pick_up_w;


// offset for piece-box storage
  double offset_storage_pose_pick_up_x;
  double offset_storage_pose_pick_up_y;
  double offset_storage_pose_pick_up_z;
  double offset_storage_orientation_pick_up_x;
  double offset_storage_orientation_pick_up_y;
  double offset_storage_orientation_pick_up_z;
  double offset_storage_orientation_pick_up_w;
*/

//---- new names

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
    offset_pick_up_orientation_x(0),offset_pick_up_orientation_y(0),offset_pick_up_orientation_z(0),offset_pick_up_orientation_w(0)
  {
    as_grasping_baxter.start();
    ar_pub = nh_.advertise<geometry_msgs::PoseStamped>("/poses/ar_code", 1);
    target_pub = nh_.advertise<geometry_msgs::PoseStamped>("/poses/target", 1);

    rightGripperPub = nh_.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command",10);


    //callback will do that later:
    ar_code_pose.header.frame_id="/world";
    ar_code_pose.header.stamp = ros::Time::now();;
    ar_code_pose.pose.position.x=0.849068284615;
    ar_code_pose.pose.position.y=-0.0577751363464;
    ar_code_pose.pose.position.z=-0.145608429006;

    ar_code_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,0,1.57);
    

 

  }

  ~grasping_baxter_boss(void)
  {
    

  }

   bool closerightGripper()
  {
    ROS_INFO("Closing right arm gripper");
    
    baxter_core_msgs::EndEffectorCommand command;
    command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
    command.args = "{\"position\": 0.0}";
    command.id = 65538;
    command.sequence = gripperSeq++;
    // Send command several times to be safe
    ROS_INFO("publishing gripper");
    rightGripperPub.publish(command);
    ros::Duration(2).sleep();
    return true;
  }

  bool openrightGripper()
  {
          ROS_INFO("Opening right arm gripper");

          baxter_core_msgs::EndEffectorCommand command;
          command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
          command.args = "{\"position\": 100.0}";
          command.id = 65538;
          command.sequence = gripperSeq++;
          rightGripperPub.publish(command);
          ros::Duration(2).sleep();
          
          ROS_INFO("Done opening");
          return true;
  }

 void ar_code_pose_callback()
 {

      p0_pose.header.stamp=ros::Time::now();
      p0_pose.header.frame_id="/world";

      p0_pose.pose.position.x=ar_code_pose.pose.position.x+offset_p0_pose_x;
      p0_pose.pose.position.y=ar_code_pose.pose.position.y+offset_p0_pose_y;
      p0_pose.pose.position.z=ar_code_pose.pose.position.z+offset_p0_pose_z;
      p0_pose.pose.orientation.x=ar_code_pose.pose.orientation.x+offset_p0_orientation_x;
      p0_pose.pose.orientation.y=ar_code_pose.pose.orientation.y+offset_p0_orientation_y;
      p0_pose.pose.orientation.z=ar_code_pose.pose.orientation.z+offset_p0_orientation_z;
      p0_pose.pose.orientation.w=ar_code_pose.pose.orientation.w+offset_p0_orientation_w;


    
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

  void place_piece(){

    //geometry_msgs::PoseStamped target_posehope;
    ar_code_pose_callback();

    target_pose.header.stamp=ros::Time::now();
    target_pose.header.frame_id="/world";
    target_pose=pick_up_pose;
    
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
    bool suckit=false;
    double offset_x=-0.065;
    double offset_y=-0.065;

    suckit= closerightGripper();
    ros::Duration(0.5).sleep();
    if(suckit){
      ROS_INFO("sucking it");

      success=false;

      int row=target_field/7;
      int col=target_field%7;

      target_pose.pose.position.x=p0_pose.pose.position.x+row*offset_x;
      target_pose.pose.position.y=p0_pose.pose.position.y+col*offset_y;

      std::cout << "reaching field n" << target_field << std::endl;;

      group.setPoseTarget(target_pose);

      success = group.plan(my_plan);

      //move it!!!
      group.move() ;
      sleep(5.0);
      ros::spinOnce();
      if(success)
        ROS_INFO("WE DID IT!!!!!!!!!!");
      else
        ROS_INFO("Fail");

      //suck up the piece
    bool stopsuckit=false;

    stopsuckit= openrightGripper();
    if(stopsuckit){
      ROS_INFO("good job");
    }
    else{
      ROS_INFO("NOO good job");
    }



    }
    else{
      ROS_INFO("Failed horribly");
    }



  }

  void picking_test(){

  //get pose

  //geometry_msgs::PoseStamped target_posehope;
    ar_code_pose_callback();


  target_pose.header.stamp=ros::Time::now();
  target_pose.header.frame_id="/world";
  target_pose=p0_pose;

    int i=0;
    double offset_x=-0.065;
    double offset_y=-0.065;
    while(i <= 48) {
        bool success=false;

        group.setPoseTarget(pick_up_pose);

        success = group.plan(my_plan);

          //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
          //move it!!!
          group.move() ;
          sleep(5.0);
          ros::spinOnce();
          if(success)
            ROS_INFO("WE DID IT!!!!!!!!!!");
          else
            ROS_INFO("Fail");


    ros::Duration(0.5).sleep();
    
success=false;
        int row=i/7;
        int col=i%7;

        target_pose.pose.position.x=p0_pose.pose.position.x+row*offset_x;
        target_pose.pose.position.y=p0_pose.pose.position.y+col*offset_y;
        target_pose.pose.position.z=pick_up_pose.pose.position.z;


          std::cout << "reaching field n" << i << std::endl;

          group.setPoseTarget(target_pose);

          success = group.plan(my_plan);

          //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
          //move it!!!
          group.move() ;
          sleep(5.0);
          ros::spinOnce();
          if(success)
            ROS_INFO("WE DID IT!!!!!!!!!!");
          else
            ROS_INFO("Fail");


    



                  i++;

   }






  }





  void grasping_baxter_environment(){
  	
  	// We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
  
  bool success =false;

  moveit::planning_interface::MoveGroup::Plan my_plan;
  geometry_msgs::PoseStamped target_posehope;
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

	/*++++++++++Table model ##############*/
	moveit_msgs::CollisionObject co_table;
	co_table.header.frame_id = group.getPlanningFrame();
	co_table.id = "Table";

	// Define a box to add to the world. 
	//shape_msgs::SolidPrimitive primitive;
	//primitive.type = primitive.BOX;
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


	all_collision_objects.push_back(co_table);



	ROS_INFO("Add an object into the world");
	//collision_objects.push_back(collision_object_table);
	planning_scene_interface.addCollisionObjects(all_collision_objects);
	ROS_INFO("Attach the object to the robot");

  	



  }

  void grasping_baxter_start_command(const grasping_baxter::grasping_baxter_game_masterGoalConstPtr &goal)
  {
  	
grasping_baxter_environment();

    // helper variables
    ros::Rate r(1);
    bool success = true;

    ROS_INFO("calculating move");
    //CALCULATE THE AI MOVE******************************

    target_field =goal->move;

    // Planning to a Pose goal
      // ^^^^^^^^^^^^^^^^^^^^^^^ 
    group.setPlannerId("RRTConnectkConfigDefault");
  
 //target_posehope= group.getRandomPose(group.getEndEffectorLink().c_str());
  
  

/*
    moveit_msgs::OrientationConstraint ocm;

    ocm.link_name = group.getEndEffectorLink().c_str();
    ocm.header.frame_id = "world";
    ocm.orientation = ar_code_pose.pose.orientation; //tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    ocm.absolute_x_axis_tolerance = 0.5;
    ocm.absolute_y_axis_tolerance = 0.5;
    ocm.absolute_z_axis_tolerance = 0.5;
    ocm.weight = 1.0;
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    //group.setPathConstraints(test_constraints);
  //goal_pub.publish(target_posehope);
  */
    ros::Duration(0.5).sleep();

    //place_piece();
    picking_test();
    /*

    group.setPoseTarget(ar_code_pose);

  success = group.plan(my_plan);

  //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
  //move it!!!
  group.move() ;
  sleep(5.0);
  ros::spinOnce();
  if(success)
    ROS_INFO("WE DID IT!!!!!!!!!!");
  else
    ROS_INFO("Fail");
    */


    //ros::spinOnce();




    //***************************************************

    // push_back the seeds for the fibonacci sequence
    feedback_grasping_baxter.progress=20; // progress in %

    // publish info to the console for the user
    ROS_INFO("%s: start received", action_name_.c_str());
	//ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);	
    
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
