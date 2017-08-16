
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <camera/camera_game_masterAction.h>
class camera_boss
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<camera::camera_game_masterAction> as_camera; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  camera::camera_game_masterFeedback feedback_camera; // create messages that are used to published feedback
  camera::camera_game_masterResult result_camera;    // create messages that are used to published result

public:

	bool camera_start_flag;

  camera_boss(std::string name) :
    as_camera(nh_, name, boost::bind(&camera_boss::camera_start_command, this, _1), false),
    action_name_(name), camera_start_flag(false)
  {
    as_camera.start();
  }

  ~camera_boss(void)
  {
  }

  void camera_start_command(const camera::camera_game_masterGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    ROS_INFO("starting camera");
    //CALCULATE THE AI MOVE******************************
    //***************************************************

    // push_back the seeds for the fibonacci sequence
    feedback_camera.sequence.clear();
    feedback_camera.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: start camera received", action_name_.c_str());
	//ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);	
    
    //fedback that everything is ok
    as_camera.publishFeedback(feedback_camera);
    

    if(success)
    {
    	ROS_INFO("camera is running");
      result_camera.sequence = feedback_camera.sequence;
      ROS_INFO("%s: Done", action_name_.c_str());
      // set the action state to succeeded
      as_camera.setSucceeded(result_camera);
      camera_start_flag=true;
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_game_master");
  ROS_INFO("Start camera node");
  //start action server
  camera_boss ab("camera_game_master");

  ros::spin();
   

  return 0;
}
