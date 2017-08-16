
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ai/ai_game_masterAction.h>
class ai_boss
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<ai::ai_game_masterAction> as_ai; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  ai::ai_game_masterFeedback feedback_ai; // create messages that are used to published feedback
  ai::ai_game_masterResult result_ai;    // create messages that are used to published result

public:

	bool ai_start_flag;

  ai_boss(std::string name) :
    as_ai(nh_, name, boost::bind(&ai_boss::ai_start_command, this, _1), false),
    action_name_(name), ai_start_flag(false)
  {
    as_ai.start();
  }

  ~ai_boss(void)
  {
  }

  void ai_start_command(const ai::ai_game_masterGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    ROS_INFO("calculating move");
    //CALCULATE THE AI MOVE******************************
    //***************************************************

    // push_back the seeds for the fibonacci sequence
    feedback_ai.sequence.clear();
    feedback_ai.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: start received", action_name_.c_str());
	//ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);	
    
    //fedback that everything is ok
    as_ai.publishFeedback(feedback_ai);
    

    if(success)
    {
    	ROS_INFO("calculating move done");
      result_ai.sequence = feedback_ai.sequence;
      ROS_INFO("%s: Done", action_name_.c_str());
      // set the action state to succeeded
      as_ai.setSucceeded(result_ai);
      ai_start_flag=true;
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ai_game_master");
  ROS_INFO("Start AI node");
  //start action server
  ai_boss ab("ai_game_master");

  ros::spin();
   

  return 0;
}
