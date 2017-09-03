/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Rviz display panel for controlling and debugging MoveIt! applications
*/

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QGroupBox>
#include <QSpinBox>
#include <QString>

#include <std_msgs/Int8.h>

#include "baxter_panel.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <gui/gui_game_masterAction.h>
#include <game_master/game_master_guiAction.h>

#include <std_msgs/String.h>

namespace gui
{
BaxterPanel::BaxterPanel(QWidget* parent) : rviz::Panel(parent), ac_gui("gui_game_master", true),
as_guistatus(nh_, "game_master_gui", boost::bind(&BaxterPanel::gui_status_reciver, this, _1), false),
action_name_("game_master_gui")
{
  //action cummunication
  ROS_INFO("Waiting for action server to start.");
  ac_gui.waitForServer();
  ROS_INFO("Action server started, sending goal.");
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
  //hlayout1->addWidget(rb_baxter_start_);
  //hlayout1->addWidget(rb_human_start_);

  hlayout1->addLayout(layoutrb);

  //create status lable
  lbl_status_ =new QLabel(this);
  lbl_baxter_says_ =new QLabel(this);

  

  // Horizontal Layout
  QHBoxLayout* hlayout2 = new QHBoxLayout;
  hlayout2->addWidget(new QLabel(QString("Baxter status:")));
  hlayout2->addWidget(lbl_status_);

  QHBoxLayout* hlayout3 = new QHBoxLayout;
  hlayout3->addWidget(new QLabel(QString("Baxter says:")));
  hlayout3->addWidget(lbl_baxter_says_);

  

  // Horizontal Layout
  // QHBoxLayout* hlayout3 = new QHBoxLayout;

  this->setStyleSheet("QGroupBox {  border: 1px solid gray; padding-top: 0px; }");

  // Group box
  QGroupBox* group_box = new QGroupBox();
  group_box->setLayout(hlayout2);
  group_box->setFlat(false);

  // Group box
  QGroupBox* group_box2 = new QGroupBox();
  group_box2->setLayout(hlayout3);
  group_box2->setFlat(false);

  // Verticle layout
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(hlayout1);
  layout->addWidget(group_box);
  layout->addWidget(group_box2);

  // layout->addLayout( hlayout3 );
  setLayout(layout);

  //set the baxter radio button to on
  rb_baxter_start_->setChecked(true);

  //action cummunication
  ROS_INFO("String gui action status server");
  as_guistatus.start();
  ROS_INFO("Action server started");

  ROS_INFO("Waiting for action server to start.");
  ac_gui.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  
  btn_publisher_ = nh_.advertise<std_msgs::Int8>("/TTTgame/guicontroll", 1);

}

void BaxterPanel::gui_status_reciver(const gui::game_master_guiGoalConstPtr &goal)
  {
    //ROS_INFO_STREAM_NAMED("baxter_gui", "RECIVING status ... *************************************************************************");
    //set the status     
    QString qstr_status=QString::fromStdString(goal->status);
    QString qstr_baxter_says=QString::fromStdString(goal->baxter_says);
    //std_msgs::String str_baxtersays=goal->baxter_says.c_str();
    lbl_status_->setText(qstr_status);
    lbl_baxter_says_->setText(qstr_baxter_says);

    //set succeded
    result_game_master_gui.got_it = 1;  // retuŕn best move between 0...48
      //ROS_INFO("%s: Done", action_name_.c_str());
      // set the action state to succeeded
      as_guistatus.setSucceeded(result_game_master_gui);

    }

void BaxterPanel::gameStart()
{
  ROS_INFO_STREAM_NAMED("baxter_gui", "Starting the game ...");
  std_msgs::Int8 msg;
  msg.data = 1;
  btn_publisher_.publish(msg);
  lbl_status_->setText("starting");

  gui_game_masterGoal goal;
  goal.first_player = 76;
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
  ROS_INFO_STREAM_NAMED("baxter_gui", "Stopping the game ...");
  
  std_msgs::Int8 msg;
  msg.data = 2;
  btn_publisher_.publish(msg);
  lbl_status_->setText("stoping");
  gui_game_masterGoal goal;
    goal.start_game = 2;
    if(rb_baxter_start_->isChecked() )
      goal.first_player = 2;
    if(rb_human_start_->isChecked() )
      goal.first_player=1;

    // Need boost::bind to pass in the 'this' pointer
    ac_gui.sendGoal(goal,
                boost::bind(&BaxterPanel::received_game_started, this, _1, _2),
                actionlib::SimpleActionClient<gui_game_masterAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<gui_game_masterAction>::SimpleFeedbackCallback());

}

void BaxterPanel::received_game_started(const actionlib::SimpleClientGoalState& state,
              const gui_game_masterResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    //ROS_INFO("Answer: %i", result->sequence.back());
    lbl_status_->setText("game started!!!");
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