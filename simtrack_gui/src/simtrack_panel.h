//************************************************
//**********Baxter 2017 Tic-Tac-Toe***************
//*******Nadine Drollinger & Michael C. Welle*****
//************************************************
//*******GUI node - simtrack_gui **************************
//************************************************

//************************************************
//Description: The gui to see the shopping cart for
// simtrack
//************************************************

#ifndef SIMTRACK_DASHBOARD__SIMTRACK_PANEL_H
#define SIMTRACK_DASHBOARD__SIMTRACK_PANEL_H

#ifndef Q_MOC_RUN
//ros
#include <ros/ros.h>
#include <rviz/panel.h>
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/String.h>
#endif
//Qt
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QRadioButton>

#include <QString>

class QLineEdit;
class QSpinBox;

namespace simtrack_gui
{
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
class SimtrackPanel : public rviz::Panel
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
  SimtrackPanel(QWidget *parent = 0);

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;

   
  int get_pos_num(double pos);
  void fill_cart();

  //callbacks
  void fuerte_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void groovy_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void hydro_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void milk_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void milk_red_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void tiger_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void chips_brown_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void chips_green_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void cornflaks_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void kleenex_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void kleenex_yellow_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void orange_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

  // Next come a couple of public Qt slots.
public Q_SLOTS:

  // Here we declare some internal slots.
protected Q_SLOTS:

  void clear_cart();

  // Then we finish up with protected member variables.
protected:
  QPushButton *btn_clear_cart_;
  QLabel *lbl_item_01_;
  QLabel *lbl_item_02_;
  QLabel *lbl_item_03_;
  QLabel *lbl_item_04_;
  QLabel *lbl_item_05_;
  QLabel *lbl_item_06_;
  QLabel *lbl_item_07_;
  QLabel *lbl_item_08_;
  QLabel *lbl_item_09_;
  QLabel *lbl_item_10_;
  QLabel *lbl_total_amount_;

  QLabel *lbl_cost_01_;
  QLabel *lbl_cost_02_;
  QLabel *lbl_cost_03_;
  QLabel *lbl_cost_04_;
  QLabel *lbl_cost_05_;
  QLabel *lbl_cost_06_;
  QLabel *lbl_cost_07_;
  QLabel *lbl_cost_08_;
  QLabel *lbl_cost_09_;
  QLabel *lbl_cost_10_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  //subscriber to pose from simtrack
  ros::Subscriber fuerte_pose_sub;
  ros::Subscriber groovy_pose_sub;
  ros::Subscriber hydro_pose_sub;
  ros::Subscriber milk_pose_sub;
  ros::Subscriber milk_red_pose_sub;
  ros::Subscriber tiger_pose_sub;
  ros::Subscriber chips_brown_pose_sub;
  ros::Subscriber chips_green_pose_sub;
  ros::Subscriber cornflaks_pose_sub;
  ros::Subscriber kleenex_pose_sub;
  ros::Subscriber kleenex_yellow_pose_sub;
  ros::Subscriber orange_pose_sub;

  //vars for all
  int consec_needed;
  double size_of_fields;
  std::vector<QString> cart_content;
  std::vector<double> cart_costs;
  ros::Duration new_object_time;
  //vars per object
  //fuerte turtel
  std::vector<int> fuerte_pos_history;
  ros::Time fuerte_off_screen_time;
  QString fuerte_name;
  double fuerte_cost;
  //groovy turtel
  std::vector<int> groovy_pos_history;
  ros::Time groovy_off_screen_time;
  QString groovy_name;
  double groovy_cost;
  //hydro turtel
  std::vector<int> hydro_pos_history;
  ros::Time hydro_off_screen_time;
  QString hydro_name;
  double hydro_cost;
  //milk
  std::vector<int> milk_pos_history;
  ros::Time milk_off_screen_time;
  QString milk_name;
  double milk_cost;
  //milk_red
  std::vector<int> milk_red_pos_history;
  ros::Time milk_red_off_screen_time;
  QString milk_red_name;
  double milk_red_cost;
  //tiger
  std::vector<int> tiger_pos_history;
  ros::Time tiger_off_screen_time;
  QString tiger_name;
  double tiger_cost;
  //chips_brown
  std::vector<int> chips_brown_pos_history;
  ros::Time chips_brown_off_screen_time;
  QString chips_brown_name;
  double chips_brown_cost;
  //chips_green
  std::vector<int> chips_green_pos_history;
  ros::Time chips_green_off_screen_time;
  QString chips_green_name;
  double chips_green_cost;
  //cornflaks
  std::vector<int> cornflaks_pos_history;
  ros::Time cornflaks_off_screen_time;
  QString cornflaks_name;
  double cornflaks_cost;
  //kleenex
  std::vector<int> kleenex_pos_history;
  ros::Time kleenex_off_screen_time;
  QString kleenex_name;
  double kleenex_cost;
  //kleenex_yellow
  std::vector<int> kleenex_yellow_pos_history;
  ros::Time kleenex_yellow_off_screen_time;
  QString kleenex_yellow_name;
  double kleenex_yellow_cost;
  //orange
  std::vector<int> orange_pos_history;
  ros::Time orange_off_screen_time;
  QString orange_name;
  double orange_cost;  
};

}  // end namespace gui
#endif  // simtrack_gui__Baxter_PANEL_H
