//************************************************
//**********Baxter 2017 Tic-Tac-Toe***************
//*******Nadine Drollinger & Michael C. Welle*****
//************************************************
//*******GUI node - gui *********
//************************************************

//************************************************
//Description: The gui to control the Baxter.
// Realised as plugin for rviz.
//************************************************

//Qt
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QGroupBox>
#include <QSpinBox>
#include <QString>
//ros
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int8.h>
#include <stdio.h>
#include <std_msgs/String.h>
//header
#include "baxter_panel.h"
//action
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <gui/gui_game_masterAction.h>
#include <game_master/game_master_guiAction.h>

//namespace
namespace gui
{
BaxterPanel::BaxterPanel(QWidget* parent) : rviz::Panel(parent), ac_gui("gui_game_master", true),
as_guistatus(nh_, "game_master_gui", boost::bind(&BaxterPanel::gui_status_reciver, this, _1), false),
action_name_("game_master_gui")
{
  //action communication
  ROS_DEBUG_NAMED("gui", "Waiting for action server to start.");
  ac_gui.waitForServer();
  ROS_DEBUG_NAMED("gui", "Action server started, sending goal.");
  // Create a push button
  btn_start_ = new QPushButton(this);
  btn_start_->setText("Start game");
  connect(btn_start_, SIGNAL(clicked()), this, SLOT(gameStart()));
  // Create a push button
  btn_stop_ = new QPushButton(this);
  btn_stop_->setText("Stop game");
  connect(btn_stop_, SIGNAL(clicked()), this, SLOT(gameStop()));
  //create radio buttons
  rb_baxter_start_ = new QRadioButton("Baxter starts", this);
  rb_human_start_ = new QRadioButton("Human starts", this);
  //layout radio buttons
  QVBoxLayout* layoutrb = new QVBoxLayout;
  layoutrb->addWidget(rb_baxter_start_);
  layoutrb->addWidget(rb_human_start_);
  // Horizontal Layout
  QHBoxLayout* hlayout1 = new QHBoxLayout;
  hlayout1->addWidget(btn_start_);
  hlayout1->addWidget(btn_stop_);
  hlayout1->addLayout(layoutrb);
  //create status and baxter says lable
  lbl_status_ =new QLabel(this);
  lbl_baxter_says_ =new QLabel(this);
  // Horizontal Layout
  QHBoxLayout* hlayout2 = new QHBoxLayout;
  hlayout2->addWidget(new QLabel(QString("Baxter status:")));
  hlayout2->addWidget(lbl_status_);
  QHBoxLayout* hlayout3 = new QHBoxLayout;
  hlayout3->addWidget(new QLabel(QString("Baxter says:")));
  hlayout3->addWidget(lbl_baxter_says_);
  this->setStyleSheet("QGroupBox {  border: 1px solid gray; padding-top: 0px; }");
  // Group box
  QGroupBox* group_box = new QGroupBox();
  group_box->setLayout(hlayout2);
  group_box->setFlat(false);
  // Group box 2
  QGroupBox* group_box2 = new QGroupBox();
  group_box2->setLayout(hlayout3);
  group_box2->setFlat(false);
  // Vertical layout
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(hlayout1);
  layout->addWidget(group_box);
  layout->addWidget(group_box2);
  // layout->addLayout( hlayout3 );
  setLayout(layout);
  //set the baxter radio button to on
  rb_baxter_start_->setChecked(true);

  //action communication
  ROS_DEBUG_NAMED("gui", "String gui action status server");
  as_guistatus.start();
  ROS_DEBUG_NAMED("gui", "Action server started");

  ROS_DEBUG_NAMED("gui", "Waiting for action server to start.");
  ac_gui.waitForServer();
  ROS_DEBUG_NAMED("gui", "Action server started, sending goal.");
}

//receive status updates from game_master
void BaxterPanel::gui_status_reciver(const gui::game_master_guiGoalConstPtr &goal)
  {
    ROS_DEBUG_NAMED("gui","Recived gui status from game_master.");
    //set the status     
    QString qstr_status=QString::fromStdString(goal->status);
    QString qstr_baxter_says=QString::fromStdString(goal->baxter_says);
    lbl_status_->setText(qstr_status);
    lbl_baxter_says_->setText(qstr_baxter_says);
    //set succeded
    result_game_master_gui.got_it = 1;  // retuŕn best move between 0...48
    ROS_DEBUG_NAMED("gui", "%s: Done", action_name_.c_str());
    // set the action state to succeeded
    as_guistatus.setSucceeded(result_game_master_gui);
  }

//game start
void BaxterPanel::gameStart()
{
  ROS_DEBUG_NAMED("gui", "Starting the game ...");
  //send start and which player starts to the game_master
  gui_game_masterGoal goal;
  goal.start_game = 1;
  if(rb_baxter_start_->isChecked())
    goal.first_player = 2;
  if(rb_human_start_->isChecked())
    goal.first_player=1;
  // Need boost::bind to pass in the 'this' pointer
  ac_gui.sendGoal(goal,
              boost::bind(&BaxterPanel::received_game_started, this, _1, _2),
              actionlib::SimpleActionClient<gui_game_masterAction>::SimpleActiveCallback(),
              actionlib::SimpleActionClient<gui_game_masterAction>::SimpleFeedbackCallback());
}

void BaxterPanel::gameStop()
{
  ROS_DEBUG_NAMED("gui", "Stopping the game ...");
  //send stop to game_master
  gui_game_masterGoal goal;
  goal.start_game = 2;
  goal.first_player = 0;
  
  // Need boost::bind to pass in the 'this' pointer
  ac_gui.sendGoal(goal,
                boost::bind(&BaxterPanel::received_game_started, this, _1, _2),
                actionlib::SimpleActionClient<gui_game_masterAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<gui_game_masterAction>::SimpleFeedbackCallback());

}

//sucedded in sending start or stop
void BaxterPanel::received_game_started(const actionlib::SimpleClientGoalState& state,
              const gui_game_masterResultConstPtr& result)
  {
    ROS_DEBUG_NAMED("gui", "Finished in state [%s]", state.toString().c_str());
  }


// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void BaxterPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}
// Load all configuration data for this panel from the given Config object.
void BaxterPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}
}  // end namespace moveit_dashboard

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gui::BaxterPanel, rviz::Panel)