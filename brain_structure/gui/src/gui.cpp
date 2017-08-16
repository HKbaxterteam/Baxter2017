#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <gui/gui_game_masterAction.h>

using namespace gui;
typedef actionlib::SimpleActionClient<gui_game_masterAction> Client;

class MyNode
{
public:
  MyNode() : ac("gui_game_master", true)
  {
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
  }

  void doStuff(int order)
  {
    gui_game_masterGoal goal;
    goal.order = order;

    // Need boost::bind to pass in the 'this' pointer
    ac.sendGoal(goal,
                boost::bind(&MyNode::doneCb, this, _1, _2),
                Client::SimpleActiveCallback(),
                Client::SimpleFeedbackCallback());

  }

  void doneCb(const actionlib::SimpleClientGoalState& state,
              const gui_game_masterResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    //ROS_INFO("Answer: %i", result->sequence.back());
    ROS_INFO("Answer: ");
    ros::shutdown();
  }

private:
  Client ac;
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_gui_game_master_class_client");
  MyNode my_node;
  my_node.doStuff(10);
  ros::spin();
  return 0;
}




































// #include <ros/ros.h>
// #include <actionlib/client/simple_action_client.h>
// #include <actionlib/client/terminal_state.h>
// //#include <actionlib_tutorials/FibonacciAction.h>
// #include <gui/gui_game_masterAction.h>

// int main (int argc, char **argv)
// {
//   ros::init(argc, argv, "send_start");

//   // create the action client
//   // true causes the client to spin its own thread
//   actionlib::SimpleActionClient<gui::gui_game_masterAction> ac("gui_game_master", true);

//   ROS_INFO("Waiting for action server to start.");
//   // wait for the action server to start
//   ac.waitForServer(); //will wait for infinite time

//   ROS_INFO("Action server started, sending  start-signal");
//   // send a goal to the action
//   gui::gui_game_masterGoal goal;
//   goal.order = 1;
//   ac.sendGoal(goal);

//   //wait for the action to return
//   bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

//   if (finished_before_timeout)
//   {
//     actionlib::SimpleClientGoalState state = ac.getState();
//     ROS_INFO("Action finished: %s",state.toString().c_str());
//   }
//   else
//     ROS_INFO("Action did not finish before the time out.");

//   //exit
//   return 0;
// }