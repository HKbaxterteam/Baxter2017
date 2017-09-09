//************************************************
//**********Baxter 2017 Tic-Tac-Toe***************
//*******Nadine Drollinger & Michael C. Welle*****
//************************************************
//*******GUI node - gui **************************
//************************************************

//************************************************
//Description: The gui to controll the Baxter.
// header stuff
//************************************************

#ifndef BAXTER_DASHBOARD__BAXTER_PANEL_H
#define BAXTER_DASHBOARD__BAXTER_PANEL_H

#ifndef Q_MOC_RUN
//ros
#include <ros/ros.h>
#include <rviz/panel.h>
#endif
//Qt
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QRadioButton>
//Action
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <gui/gui_game_masterAction.h>
#include <gui/game_master_guiAction.h>

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

  void groundState();
  
  void checkReach();

  void received_game_started(const actionlib::SimpleClientGoalState& state,
              const gui_game_masterResultConstPtr& result);

  void gui_status_reciver(const gui::game_master_guiGoalConstPtr &goal);

  // Then we finish up with protected member variables.
protected:
  QPushButton *btn_start_;
  QPushButton *btn_stop_;
  QPushButton *btn_ground_state_;
  QPushButton *btn_check_reach_;
  QLabel *lbl_status_;
  QLabel *lbl_baxter_says_;
  QRadioButton *rb_baxter_start_;
  QRadioButton *rb_human_start_;
  // The ROS node handle.
  ros::NodeHandle nh_;

  //action client
  actionlib::SimpleActionClient<gui_game_masterAction> ac_gui;
  //action  
  actionlib::SimpleActionServer<gui::game_master_guiAction> as_guistatus; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  gui::game_master_guiFeedback feedback_game_master_gui; // create messages that are used to published feedback
  gui::game_master_guiResult result_game_master_gui;    // create messages that are used to published result
  std::string action_name_;
};

}  // end namespace gui

#endif  // gui__Baxter_PANEL_H
