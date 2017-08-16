
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <grasping_baxter/grasping_baxter_game_masterAction.h>
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

  grasping_baxter_boss(std::string name) :
    as_grasping_baxter(nh_, name, boost::bind(&grasping_baxter_boss::grasping_baxter_start_command, this, _1), false),
    action_name_(name), grasping_baxter_start_flag(false)
  {
    as_grasping_baxter.start();
  }

  ~grasping_baxter_boss(void)
  {
  }

  void grasping_baxter_start_command(const grasping_baxter::grasping_baxter_game_masterGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    ROS_INFO("calculating move");
    //CALCULATE THE AI MOVE******************************
    //***************************************************

    // push_back the seeds for the fibonacci sequence
    feedback_grasping_baxter.sequence.clear();
    feedback_grasping_baxter.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: start received", action_name_.c_str());
	//ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);	
    
    //fedback that everything is ok
    as_grasping_baxter.publishFeedback(feedback_grasping_baxter);
    

    if(success)
    {
    	ROS_INFO("executing move ");
      result_grasping_baxter.sequence = feedback_grasping_baxter.sequence;
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
  grasping_baxter_boss ab("grasping_baxter_game_master");

  ros::spin();
   

  return 0;
}
