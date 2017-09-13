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
//
// MoveItPanel will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// DriveWidget class, and is described there.
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
  void milk_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void tiger_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void instant_green_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void instant_blue_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void chips_brown_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void chips_green_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void coke_can_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void m_dew_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void can_corn_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void can_borsch_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void cornflaks_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);


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
  ros::Subscriber milk_pose_sub;
  ros::Subscriber tiger_pose_sub;
  ros::Subscriber instant_green_pose_sub;
  ros::Subscriber instant_blue_pose_sub;
  ros::Subscriber chips_brown_pose_sub;
  ros::Subscriber chips_green_pose_sub;
  ros::Subscriber coke_can_pose_sub;
  ros::Subscriber m_dew_pose_sub;
  ros::Subscriber can_corn_pose_sub;
  ros::Subscriber can_borsch_pose_sub;
  ros::Subscriber cornflaks_pose_sub;


  //
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
  //milk
  std::vector<int> milk_pos_history;
  ros::Time milk_off_screen_time;
  QString milk_name;
  double milk_cost;
  //tiger
  std::vector<int> tiger_pos_history;
  ros::Time tiger_off_screen_time;
  QString tiger_name;
  double tiger_cost;
  //instant_green
  std::vector<int> instant_green_pos_history;
  ros::Time instant_green_off_screen_time;
  QString instant_green_name;
  double instant_green_cost;
  //instant_blue
  std::vector<int> instant_blue_pos_history;
  ros::Time instant_blue_off_screen_time;
  QString instant_blue_name;
  double instant_blue_cost;
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
  //coke_can
  std::vector<int> coke_can_pos_history;
  ros::Time coke_can_off_screen_time;
  QString coke_can_name;
  double coke_can_cost;
  //m_dew
  std::vector<int> m_dew_pos_history;
  ros::Time m_dew_off_screen_time;
  QString m_dew_name;
  double m_dew_cost;
  //can_corn
  std::vector<int> can_corn_pos_history;
  ros::Time can_corn_off_screen_time;
  QString can_corn_name;
  double can_corn_cost;
  //can_borsch
  std::vector<int> can_borsch_pos_history;
  ros::Time can_borsch_off_screen_time;
  QString can_borsch_name;
  double can_borsch_cost;
  //cornflaks
  std::vector<int> cornflaks_pos_history;
  ros::Time cornflaks_off_screen_time;
  QString cornflaks_name;
  double cornflaks_cost;

  

  
};

}  // end namespace gui

#endif  // simtrack_gui__Baxter_PANEL_H
