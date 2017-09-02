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

// TODO: convert to flow layout:
// http://doc.qt.io/qt-5/qtwidgets-layouts-flowlayout-example.html

#ifndef BAXTER_DASHBOARD__BAXTER_PANEL_H
#define BAXTER_DASHBOARD__BAXTER_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#endif

#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QRadioButton>

   #include <actionlib/client/simple_action_client.h>
#include <gui/gui_game_masterAction.h>

class QLineEdit;
class QSpinBox;

namespace gui
{
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
// MoveItPanel will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// DriveWidget class, and is described there.
class BaxterPanel : public rviz::Panel
{
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  BaxterPanel(QWidget *parent = 0);

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:

  // Here we declare some internal slots.
protected Q_SLOTS:

  void gameStart();

  void gameStop();

  void received_game_started(const actionlib::SimpleClientGoalState& state,
              const gui_game_masterResultConstPtr& result);

  // Then we finish up with protected member variables.
protected:
  QPushButton *btn_start_;
  QPushButton *btn_stop_;
  QLabel *lbl_status_;
  QRadioButton *rb_baxter_start_;
  QRadioButton *rb_human_start_;

  //action client
  actionlib::SimpleActionClient<gui_game_masterAction> ac_gui;
  // The ROS publishers
  ros::Publisher btn_publisher_;

  // The ROS node handle.
  ros::NodeHandle nh_;
};

}  // end namespace gui

#endif  // gui__Baxter_PANEL_H
