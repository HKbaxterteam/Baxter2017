
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
	ros::Publisher goal_pub;
	geometry_msgs::PoseStamped ar_code_pose;



  grasping_baxter_boss(std::string name) :
    as_grasping_baxter(nh_, name, boost::bind(&grasping_baxter_boss::grasping_baxter_start_command, this, _1), false),
    action_name_(name), grasping_baxter_start_flag(false),group("right_arm")
  {
    as_grasping_baxter.start();
    goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("goalpose", 1);


    //callback will do that later:
    ar_code_pose.header.frame_id="/world";
    ar_code_pose.header.stamp = ros::Time::now();;
    ar_code_pose.pose.position.x=0.45;
    ar_code_pose.pose.position.y=-0.2;
    ar_code_pose.pose.position.z=-0.15;
    ar_code_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,0,1.57);
    
    

  }

  ~grasping_baxter_boss(void)
  {
    

  }

  void publish_goalpose(){
  	goal_pub.publish(ar_code_pose);

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
  group.setGoalOrientationTolerance(0.01);
  group.setGoalPositionTolerance(0.01);
  group.setGoalTolerance(0.01);
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
	cube.dimensions[2] = 0.75;

	// A pose for the box (specified relative to frame_id) 
	geometry_msgs::Pose table_pose;
	table_pose.orientation.w = 1.0;
	table_pose.position.x =  0.825;
	table_pose.position.y = 0;
	table_pose.position.z =  -0.575;

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

    // Planning to a Pose goal
      // ^^^^^^^^^^^^^^^^^^^^^^^ 
    group.setPlannerId("RRTConnectkConfigDefault");
  
 target_posehope= group.getRandomPose(group.getEndEffectorLink().c_str());
  
  group.setPoseTarget(ar_code_pose);

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
    ros::Duration(0.5).sleep();

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
  	ROS_INFO("publishing");
  	ros::spinOnce();
  	ros::Duration(0.5).sleep(); 
  }

  ros::spin();


}
