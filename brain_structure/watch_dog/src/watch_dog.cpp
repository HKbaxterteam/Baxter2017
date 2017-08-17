
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <watch_dog/watch_dog_game_masterAction.h>
class watch_dog_boss
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<watch_dog::watch_dog_game_masterAction> as_watch_dog; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  watch_dog::watch_dog_game_masterFeedback feedback_watch_dog; // create messages that are used to published feedback
  watch_dog::watch_dog_game_masterResult result_watch_dog;    // create messages that are used to published result

public:

	bool watch_dog_start_flag;

  watch_dog_boss(std::string name) :
    as_watch_dog(nh_, name, boost::bind(&watch_dog_boss::watch_dog_start_command, this, _1), false),
    action_name_(name), watch_dog_start_flag(false)
  {
    as_watch_dog.start();
  }

  ~watch_dog_boss(void)
  {
  }

  void watch_dog_start_command(const watch_dog::watch_dog_game_masterGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    ROS_INFO("starting watch_dog");
    //CALCULATE THE AI MOVE******************************
    //***************************************************

    // push_back the seeds for the fibonacci sequence
    feedback_watch_dog.progress=20; // progress in %

    // publish info to the console for the user
    ROS_INFO("%s: start watch_dog received", action_name_.c_str());
	//ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);	
    
    //fedback that everything is ok
    as_watch_dog.publishFeedback(feedback_watch_dog);
    

    if(success)
    {
    	ROS_INFO("watch_dog is running");
      result_watch_dog.watch_dog_done = 1;
      ROS_INFO("%s: Done", action_name_.c_str());
      // set the action state to succeeded
      as_watch_dog.setSucceeded(result_watch_dog);
      watch_dog_start_flag=true;
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "watch_dog_game_master");
  ROS_INFO("Start watch_dog node");
  //start action server
  watch_dog_boss ab("watch_dog_game_master");

  ros::spin();
   

  return 0;
}
