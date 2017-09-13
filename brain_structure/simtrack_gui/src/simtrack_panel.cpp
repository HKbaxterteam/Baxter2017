//************************************************
//**********Baxter 2017 Tic-Tac-Toe***************
//*******Nadine Drollinger & Michael C. Welle*****
//************************************************
//*******GUI node - Simtrack_gui *********
//************************************************

//************************************************
//Description: The gui to see the shopping cart for
// simtrack
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
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <iostream>
//header
#include "simtrack_panel.h"

//namespace
namespace simtrack_gui
{
SimtrackPanel::SimtrackPanel(QWidget* parent) : rviz::Panel(parent),consec_needed(5),size_of_fields(0.077),new_object_time(5.0),
fuerte_name("Fuerte turtel"),fuerte_cost(8.77),milk_name("Skimmed milk"),milk_cost(26.15),tiger_name("Kellogs's super suger"),tiger_cost(29.85),
instant_green_name("Damae Iccho Tonkoitsu noodle"),instant_green_cost(6.45),instant_blue_name("Cup noodles mini"),instant_blue_cost(4.20),
chips_brown_name("Pringles Hot & Spicy"),chips_brown_cost(19.78),chips_green_name("Pringles Sour cream & Onion"),chips_green_cost(19.78),
coke_can_name("Coka Cola can"),coke_can_cost(6.69),m_dew_name("Mountam Dew"),m_dew_cost(7.77),can_corn_name("Cream Style Corn"),can_corn_cost(3.25),
can_borsch_name("Campbells Borsch"),can_borsch_cost(4.89),cornflaks_name("Kelloggs Cornflaks"),cornflaks_cost(45.30)
{
  // Create a push button
  btn_clear_cart_ = new QPushButton(this);
  btn_clear_cart_->setText("Clear_cart");
  connect(btn_clear_cart_, SIGNAL(clicked()), this, SLOT(clear_cart()));

  //create the cart layout
  lbl_item_01_ =new QLabel(this);
  lbl_item_02_ =new QLabel(this);
  lbl_item_03_ =new QLabel(this);
  lbl_item_04_ =new QLabel(this);
  lbl_item_05_ =new QLabel(this);
  lbl_item_06_ =new QLabel(this);
  lbl_item_07_ =new QLabel(this);
  lbl_item_08_ =new QLabel(this);
  lbl_item_09_ =new QLabel(this);
  lbl_item_10_ =new QLabel(this);
  lbl_cost_01_ =new QLabel(this);
  lbl_cost_02_ =new QLabel(this);
  lbl_cost_03_ =new QLabel(this);
  lbl_cost_04_ =new QLabel(this);
  lbl_cost_05_ =new QLabel(this);
  lbl_cost_06_ =new QLabel(this);
  lbl_cost_07_ =new QLabel(this);
  lbl_cost_08_ =new QLabel(this);
  lbl_cost_09_ =new QLabel(this);
  lbl_cost_10_ =new QLabel(this);
  lbl_total_amount_ =new QLabel(this);

  // Horizontal Layouts
  QHBoxLayout* hlayout0 = new QHBoxLayout;
  hlayout0->addWidget(new QLabel(QString("Shopping cart: ")));
  hlayout0->addWidget(btn_clear_cart_);
  QHBoxLayout* hlayout0a = new QHBoxLayout;
  hlayout0a->addWidget(new QLabel(QString("Item")));
  hlayout0a->addWidget(new QLabel(QString("|")));
  hlayout0a->addWidget(new QLabel(QString("Cost")));
  QHBoxLayout* hlayout1 = new QHBoxLayout;
  hlayout1->addWidget(lbl_item_01_);
  hlayout1->addWidget(new QLabel(QString("|")));
  hlayout1->addWidget(lbl_cost_01_);
  QHBoxLayout* hlayout2 = new QHBoxLayout;
  hlayout2->addWidget(lbl_item_02_);
  hlayout2->addWidget(new QLabel(QString("|")));
  hlayout2->addWidget(lbl_cost_02_);
  QHBoxLayout* hlayout3 = new QHBoxLayout;
  hlayout3->addWidget(lbl_item_03_);
  hlayout3->addWidget(new QLabel(QString("|")));
  hlayout3->addWidget(lbl_cost_03_);
  QHBoxLayout* hlayout4 = new QHBoxLayout;
  hlayout4->addWidget(lbl_item_04_);
  hlayout4->addWidget(new QLabel(QString("|")));
  hlayout4->addWidget(lbl_cost_04_);
  QHBoxLayout* hlayout5 = new QHBoxLayout;
  hlayout5->addWidget(lbl_item_05_);
  hlayout5->addWidget(new QLabel(QString("|")));
  hlayout5->addWidget(lbl_cost_05_);
  QHBoxLayout* hlayout6 = new QHBoxLayout;
  hlayout6->addWidget(lbl_item_06_);
  hlayout6->addWidget(new QLabel(QString("|")));
  hlayout6->addWidget(lbl_cost_06_);
  QHBoxLayout* hlayout7 = new QHBoxLayout;
  hlayout7->addWidget(lbl_item_07_);
  hlayout7->addWidget(new QLabel(QString("|")));
  hlayout7->addWidget(lbl_cost_07_);
  QHBoxLayout* hlayout8 = new QHBoxLayout;
  hlayout8->addWidget(lbl_item_08_);
  hlayout8->addWidget(new QLabel(QString("|")));
  hlayout8->addWidget(lbl_cost_08_);
  QHBoxLayout* hlayout9 = new QHBoxLayout;
  hlayout9->addWidget(lbl_item_09_);
  hlayout9->addWidget(new QLabel(QString("|")));
  hlayout9->addWidget(lbl_cost_09_);
  QHBoxLayout* hlayout10 = new QHBoxLayout;
  hlayout10->addWidget(lbl_item_10_);
  hlayout10->addWidget(new QLabel(QString("|")));
  hlayout10->addWidget(lbl_cost_10_);
  QHBoxLayout* hlayout11 = new QHBoxLayout;
  hlayout11->addWidget(new QLabel(QString("Total amount: ")));
  hlayout11->addWidget(new QLabel(QString(" ")));
  hlayout11->addWidget(lbl_total_amount_);
  this->setStyleSheet("QGroupBox {  border: 1px solid gray; padding-top: 0px; }");

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(hlayout0);
  layout->addWidget(new QLabel(QString("_______________________________________________")));
  layout->addLayout(hlayout0a);
  layout->addWidget(new QLabel(QString("-----------------------------------------------")));
  layout->addLayout(hlayout1);
  layout->addLayout(hlayout2);
  layout->addLayout(hlayout3);
  layout->addLayout(hlayout4);
  layout->addLayout(hlayout5);
  layout->addLayout(hlayout6);
  layout->addLayout(hlayout7);
  layout->addLayout(hlayout8);
  layout->addLayout(hlayout9);
  layout->addLayout(hlayout10);
  layout->addWidget(new QLabel(QString("_______________________________________________")));
  layout->addLayout(hlayout11);

  // layout->addLayout( hlayout3 );
  setLayout(layout);

  //subscribers
  fuerte_pose_sub = nh_.subscribe("/simtrack/ros_fuerte", 1, &SimtrackPanel::fuerte_code_pose_cb, this);
  fuerte_off_screen_time=ros::Time::now();
  milk_pose_sub = nh_.subscribe("/simtrack/milk", 1, &SimtrackPanel::milk_code_pose_cb, this);
  milk_off_screen_time=ros::Time::now();
  tiger_pose_sub = nh_.subscribe("/simtrack/tiger", 1, &SimtrackPanel::tiger_code_pose_cb, this);
  tiger_off_screen_time=ros::Time::now();
  instant_green_pose_sub = nh_.subscribe("/simtrack/instant_green", 1, &SimtrackPanel::instant_green_code_pose_cb, this);
  instant_green_off_screen_time=ros::Time::now();
  instant_blue_pose_sub = nh_.subscribe("/simtrack/instant_blue", 1, &SimtrackPanel::instant_blue_code_pose_cb, this);
  instant_blue_off_screen_time=ros::Time::now();
  chips_brown_pose_sub = nh_.subscribe("/simtrack/chips_brown", 1, &SimtrackPanel::chips_brown_code_pose_cb, this);
  chips_brown_off_screen_time=ros::Time::now();
  chips_green_pose_sub = nh_.subscribe("/simtrack/chips_green", 1, &SimtrackPanel::chips_green_code_pose_cb, this);
  chips_green_off_screen_time=ros::Time::now();
  coke_can_pose_sub = nh_.subscribe("/simtrack/coke_can", 1, &SimtrackPanel::coke_can_code_pose_cb, this);
  coke_can_off_screen_time=ros::Time::now();
  m_dew_pose_sub = nh_.subscribe("/simtrack/m_dew", 1, &SimtrackPanel::m_dew_code_pose_cb, this);
  m_dew_off_screen_time=ros::Time::now();
  can_corn_pose_sub = nh_.subscribe("/simtrack/can_corn", 1, &SimtrackPanel::can_corn_code_pose_cb, this);
  can_corn_off_screen_time=ros::Time::now();
  can_borsch_pose_sub = nh_.subscribe("/simtrack/can_borsch", 1, &SimtrackPanel::can_borsch_code_pose_cb, this);
  can_borsch_off_screen_time=ros::Time::now();
  cornflaks_pose_sub = nh_.subscribe("/simtrack/cornflaks", 1, &SimtrackPanel::cornflaks_code_pose_cb, this);
  cornflaks_off_screen_time=ros::Time::now();

  
}

void SimtrackPanel::fuerte_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    ROS_DEBUG_THROTTLE(5,"Pose Callback");
    //abourt if to close
    if(msg->pose.position.z<0.3)
      return;
    //we get callback when we track the thing ... hopefully ... 
    //=> initilize object when callback comes and reset if no callback in t secs
    //check if the thing is on screen
    if(ros::Time::now()>=fuerte_off_screen_time+new_object_time){
      //it was gone to long
      fuerte_pos_history.clear();
      ROS_DEBUG_NAMED("simtrack_gui", "New object detected: fuerte");
    }
    //update time
    fuerte_off_screen_time=ros::Time::now();
    //get the movenumber
    int pos_num = get_pos_num(msg->pose.position.x);
    //first entry
    if(fuerte_pos_history.size()==0){
      fuerte_pos_history.push_back(pos_num);
    }
    //if we have the same pos
    if(fuerte_pos_history[fuerte_pos_history.size()-1]==pos_num){
      //ROS_INFO("same");
    }
    //new pose
    if(fuerte_pos_history[fuerte_pos_history.size()-1]!=pos_num){
      fuerte_pos_history.push_back(pos_num);
      ROS_DEBUG_NAMED("simtrack_gui", "new move fuerte %i",pos_num);
    }
    int temp=fuerte_pos_history.size();
    //check if we have a movement
    //check if there is enough data to do movment check
    if(fuerte_pos_history.size()<consec_needed+1){
      //ROS_INFO("to fee pos");
      return;
    }
    //look for movement to the right
    int count=0;
    for(int i=0;i<fuerte_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(fuerte_pos_history[i+j]+1==fuerte_pos_history[i+j+1]){
          count++;
        }
      }
        //do we move to the right?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to the right detected");
        cart_content.push_back(fuerte_name);
        cart_costs.push_back(fuerte_cost);
        fill_cart();
        //delete history as we have the object
        fuerte_pos_history.clear();
        return;
      }
      count=0;
    }
    
    //look for movement to the left
    count=0;
    for(int i=0;i<fuerte_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(fuerte_pos_history[i+j]-1==fuerte_pos_history[i+j+1]){
          count++;
        }
      }
      //do we move to the left?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to left detected");
        //find pos of entry
        int del_pos=-1;
        for(int i=0;i<cart_content.size();i++){
          if(cart_content[i]==fuerte_name)
            del_pos=i;
        }
        if(del_pos==-1){
          fuerte_pos_history.clear();
          return;
        }
        else{

          
          cart_content.erase(cart_content.begin() + del_pos);
          cart_costs.erase(cart_costs.begin() + del_pos);
          fill_cart();
          //delete history as we have the object
          fuerte_pos_history.clear();
          return;
        } 
        count=0;
      }         
    }
  }

  void SimtrackPanel::milk_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    ROS_DEBUG_THROTTLE(5,"Pose Callback");
    //abourt if to close
    if(msg->pose.position.z<0.3)
      return;
    //we get callback when we track the thing ... hopefully ... 
    //=> initilize object when callback comes and reset if no callback in t secs
    //check if the thing is on screen
    if(ros::Time::now()>=milk_off_screen_time+new_object_time){
      //it was gone to long
      milk_pos_history.clear();
      ROS_DEBUG_NAMED("simtrack_gui", "New object detected: milk");
    }
    //update time
    milk_off_screen_time=ros::Time::now();
    //get the movenumber
    int pos_num = get_pos_num(msg->pose.position.x);
    //first entry
    if(milk_pos_history.size()==0){
      milk_pos_history.push_back(pos_num);
    }
    //if we have the same pos
    if(milk_pos_history[milk_pos_history.size()-1]==pos_num){
      //ROS_INFO("same");
    }
    //new pose
    if(milk_pos_history[milk_pos_history.size()-1]!=pos_num){
      milk_pos_history.push_back(pos_num);
      ROS_DEBUG_NAMED("simtrack_gui", "new move milk %i",pos_num);
    }
    int temp=milk_pos_history.size();
    //check if we have a movement
    //check if there is enough data to do movment check
    if(milk_pos_history.size()<consec_needed+1){
      //ROS_INFO("to fee pos");
      return;
    }
    //look for movement to the right
    int count=0;
    for(int i=0;i<milk_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(milk_pos_history[i+j]+1==milk_pos_history[i+j+1]){
          count++;
        }
      }
        //do we move to the right?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to the right detected");
        cart_content.push_back(milk_name);
        cart_costs.push_back(milk_cost);
        fill_cart();
        //delete history as we have the object
        milk_pos_history.clear();
        return;
      }
      count=0;
    }
    
    //look for movement to the left
    count=0;
    for(int i=0;i<milk_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(milk_pos_history[i+j]-1==milk_pos_history[i+j+1]){
          count++;
        }
      }
      //do we move to the left?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to left detected");
        //find pos of entry
        int del_pos=-1;
        for(int i=0;i<cart_content.size();i++){
          if(cart_content[i]==milk_name)
            del_pos=i;
        }
        if(del_pos==-1){
          milk_pos_history.clear();
          return;
        }
        else{

          
          cart_content.erase(cart_content.begin() + del_pos);
          cart_costs.erase(cart_costs.begin() + del_pos);
          fill_cart();
          //delete history as we have the object
          milk_pos_history.clear();
          return;
        } 
        count=0;
      }         
    }
  }

   void SimtrackPanel::tiger_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    ROS_DEBUG_THROTTLE(5,"Pose Callback");
    //abourt if to close
    if(msg->pose.position.z<0.3)
      return;
    //we get callback when we track the thing ... hopefully ... 
    //=> initilize object when callback comes and reset if no callback in t secs
    //check if the thing is on screen
    if(ros::Time::now()>=tiger_off_screen_time+new_object_time){
      //it was gone to long
      tiger_pos_history.clear();
      ROS_DEBUG_NAMED("simtrack_gui", "New object detected: tiger");
    }
    //update time
    tiger_off_screen_time=ros::Time::now();
    //get the movenumber
    int pos_num = get_pos_num(msg->pose.position.x);
    //first entry
    if(tiger_pos_history.size()==0){
      tiger_pos_history.push_back(pos_num);
    }
    //if we have the same pos
    if(tiger_pos_history[tiger_pos_history.size()-1]==pos_num){
      //ROS_INFO("same");
    }
    //new pose
    if(tiger_pos_history[tiger_pos_history.size()-1]!=pos_num){
      tiger_pos_history.push_back(pos_num);
      ROS_DEBUG_NAMED("simtrack_gui", "new move tiger %i",pos_num);
    }
    int temp=tiger_pos_history.size();
    //check if we have a movement
    //check if there is enough data to do movment check
    if(tiger_pos_history.size()<consec_needed+1){
      //ROS_INFO("to fee pos");
      return;
    }
    //look for movement to the right
    int count=0;
    for(int i=0;i<tiger_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(tiger_pos_history[i+j]+1==tiger_pos_history[i+j+1]){
          count++;
        }
      }
        //do we move to the right?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to the right detected");
        cart_content.push_back(tiger_name);
        cart_costs.push_back(tiger_cost);
        fill_cart();
        //delete history as we have the object
        tiger_pos_history.clear();
        return;
      }
      count=0;
    }
    
    //look for movement to the left
    count=0;
    for(int i=0;i<tiger_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(tiger_pos_history[i+j]-1==tiger_pos_history[i+j+1]){
          count++;
        }
      }
      //do we move to the left?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to left detected");
        //find pos of entry
        int del_pos=-1;
        for(int i=0;i<cart_content.size();i++){
          if(cart_content[i]==tiger_name)
            del_pos=i;
        }
        if(del_pos==-1){
          tiger_pos_history.clear();
          return;
        }
        else{

          
          cart_content.erase(cart_content.begin() + del_pos);
          cart_costs.erase(cart_costs.begin() + del_pos);
          fill_cart();
          //delete history as we have the object
          tiger_pos_history.clear();
          return;
        } 
        count=0;
      }         
    }
  }

   void SimtrackPanel::instant_blue_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    ROS_DEBUG_THROTTLE(5,"Pose Callback");
    //abourt if to close
    if(msg->pose.position.z<0.3)
      return;
    //we get callback when we track the thing ... hopefully ... 
    //=> initilize object when callback comes and reset if no callback in t secs
    //check if the thing is on screen
    if(ros::Time::now()>=instant_blue_off_screen_time+new_object_time){
      //it was gone to long
      instant_blue_pos_history.clear();
      ROS_DEBUG_NAMED("simtrack_gui", "New object detected: instant_blue");
    }
    //update time
    instant_blue_off_screen_time=ros::Time::now();
    //get the movenumber
    int pos_num = get_pos_num(msg->pose.position.x);
    //first entry
    if(instant_blue_pos_history.size()==0){
      instant_blue_pos_history.push_back(pos_num);
    }
    //if we have the same pos
    if(instant_blue_pos_history[instant_blue_pos_history.size()-1]==pos_num){
      //ROS_INFO("same");
    }
    //new pose
    if(instant_blue_pos_history[instant_blue_pos_history.size()-1]!=pos_num){
      instant_blue_pos_history.push_back(pos_num);
      ROS_DEBUG_NAMED("simtrack_gui", "new move instant_blue %i",pos_num);
    }
    int temp=instant_blue_pos_history.size();
    //check if we have a movement
    //check if there is enough data to do movment check
    if(instant_blue_pos_history.size()<consec_needed+1){
      //ROS_INFO("to fee pos");
      return;
    }
    //look for movement to the right
    int count=0;
    for(int i=0;i<instant_blue_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(instant_blue_pos_history[i+j]+1==instant_blue_pos_history[i+j+1]){
          count++;
        }
      }
        //do we move to the right?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to the right detected");
        cart_content.push_back(instant_blue_name);
        cart_costs.push_back(instant_blue_cost);
        fill_cart();
        //delete history as we have the object
        instant_blue_pos_history.clear();
        return;
      }
      count=0;
    }
    
    //look for movement to the left
    count=0;
    for(int i=0;i<instant_blue_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(instant_blue_pos_history[i+j]-1==instant_blue_pos_history[i+j+1]){
          count++;
        }
      }
      //do we move to the left?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to left detected");
        //find pos of entry
        int del_pos=-1;
        for(int i=0;i<cart_content.size();i++){
          if(cart_content[i]==instant_blue_name)
            del_pos=i;
        }
        if(del_pos==-1){
          instant_blue_pos_history.clear();
          return;
        }
        else{

          
          cart_content.erase(cart_content.begin() + del_pos);
          cart_costs.erase(cart_costs.begin() + del_pos);
          fill_cart();
          //delete history as we have the object
          instant_blue_pos_history.clear();
          return;
        } 
        count=0;
      }         
    }
  }


   void SimtrackPanel::instant_green_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    ROS_DEBUG_THROTTLE(5,"Pose Callback");
    //abourt if to close
    if(msg->pose.position.z<0.3)
      return;
    //we get callback when we track the thing ... hopefully ... 
    //=> initilize object when callback comes and reset if no callback in t secs
    //check if the thing is on screen
    if(ros::Time::now()>=instant_green_off_screen_time+new_object_time){
      //it was gone to long
      instant_green_pos_history.clear();
      ROS_DEBUG_NAMED("simtrack_gui", "New object detected: instant_green");
    }
    //update time
    instant_green_off_screen_time=ros::Time::now();
    //get the movenumber
    int pos_num = get_pos_num(msg->pose.position.x);
    //first entry
    if(instant_green_pos_history.size()==0){
      instant_green_pos_history.push_back(pos_num);
    }
    //if we have the same pos
    if(instant_green_pos_history[instant_green_pos_history.size()-1]==pos_num){
      //ROS_INFO("same");
    }
    //new pose
    if(instant_green_pos_history[instant_green_pos_history.size()-1]!=pos_num){
      instant_green_pos_history.push_back(pos_num);
      ROS_DEBUG_NAMED("simtrack_gui", "new move instant_green %i",pos_num);
    }
    int temp=instant_green_pos_history.size();
    //check if we have a movement
    //check if there is enough data to do movment check
    if(instant_green_pos_history.size()<consec_needed+1){
      //ROS_INFO("to fee pos");
      return;
    }
    //look for movement to the right
    int count=0;
    for(int i=0;i<instant_green_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(instant_green_pos_history[i+j]+1==instant_green_pos_history[i+j+1]){
          count++;
        }
      }
        //do we move to the right?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to the right detected");
        cart_content.push_back(instant_green_name);
        cart_costs.push_back(instant_green_cost);
        fill_cart();
        //delete history as we have the object
        instant_green_pos_history.clear();
        return;
      }
      count=0;
    }
    
    //look for movement to the left
    count=0;
    for(int i=0;i<instant_green_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(instant_green_pos_history[i+j]-1==instant_green_pos_history[i+j+1]){
          count++;
        }
      }
      //do we move to the left?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to left detected");
        //find pos of entry
        int del_pos=-1;
        for(int i=0;i<cart_content.size();i++){
          if(cart_content[i]==instant_green_name)
            del_pos=i;
        }
        if(del_pos==-1){
          instant_green_pos_history.clear();
          return;
        }
        else{

          
          cart_content.erase(cart_content.begin() + del_pos);
          cart_costs.erase(cart_costs.begin() + del_pos);
          fill_cart();
          //delete history as we have the object
          instant_green_pos_history.clear();
          return;
        } 
        count=0;
      }         
    }
  }


   void SimtrackPanel::chips_brown_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    ROS_DEBUG_THROTTLE(5,"Pose Callback");
    //abourt if to close
    if(msg->pose.position.z<0.3)
      return;
    //we get callback when we track the thing ... hopefully ... 
    //=> initilize object when callback comes and reset if no callback in t secs
    //check if the thing is on screen
    if(ros::Time::now()>=chips_brown_off_screen_time+new_object_time){
      //it was gone to long
      chips_brown_pos_history.clear();
      ROS_DEBUG_NAMED("simtrack_gui", "New object detected: chips_brown");
    }
    //update time
    chips_brown_off_screen_time=ros::Time::now();
    //get the movenumber
    int pos_num = get_pos_num(msg->pose.position.x);
    //first entry
    if(chips_brown_pos_history.size()==0){
      chips_brown_pos_history.push_back(pos_num);
    }
    //if we have the same pos
    if(chips_brown_pos_history[chips_brown_pos_history.size()-1]==pos_num){
      //ROS_INFO("same");
    }
    //new pose
    if(chips_brown_pos_history[chips_brown_pos_history.size()-1]!=pos_num){
      chips_brown_pos_history.push_back(pos_num);
      ROS_DEBUG_NAMED("simtrack_gui", "new move chips_brown %i",pos_num);
    }
    int temp=chips_brown_pos_history.size();
    //check if we have a movement
    //check if there is enough data to do movment check
    if(chips_brown_pos_history.size()<consec_needed+1){
      //ROS_INFO("to fee pos");
      return;
    }
    //look for movement to the right
    int count=0;
    for(int i=0;i<chips_brown_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(chips_brown_pos_history[i+j]+1==chips_brown_pos_history[i+j+1]){
          count++;
        }
      }
        //do we move to the right?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to the right detected");
        cart_content.push_back(chips_brown_name);
        cart_costs.push_back(chips_brown_cost);
        fill_cart();
        //delete history as we have the object
        chips_brown_pos_history.clear();
        return;
      }
      count=0;
    }
    
    //look for movement to the left
    count=0;
    for(int i=0;i<chips_brown_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(chips_brown_pos_history[i+j]-1==chips_brown_pos_history[i+j+1]){
          count++;
        }
      }
      //do we move to the left?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to left detected");
        //find pos of entry
        int del_pos=-1;
        for(int i=0;i<cart_content.size();i++){
          if(cart_content[i]==chips_brown_name)
            del_pos=i;
        }
        if(del_pos==-1){
          chips_brown_pos_history.clear();
          return;
        }
        else{

          
          cart_content.erase(cart_content.begin() + del_pos);
          cart_costs.erase(cart_costs.begin() + del_pos);
          fill_cart();
          //delete history as we have the object
          chips_brown_pos_history.clear();
          return;
        } 
        count=0;
      }         
    }
  }



   void SimtrackPanel::chips_green_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    ROS_DEBUG_THROTTLE(5,"Pose Callback");
    //abourt if to close
    if(msg->pose.position.z<0.3)
      return;
    //we get callback when we track the thing ... hopefully ... 
    //=> initilize object when callback comes and reset if no callback in t secs
    //check if the thing is on screen
    if(ros::Time::now()>=chips_green_off_screen_time+new_object_time){
      //it was gone to long
      chips_green_pos_history.clear();
      ROS_DEBUG_NAMED("simtrack_gui", "New object detected: chips_green");
    }
    //update time
    chips_green_off_screen_time=ros::Time::now();
    //get the movenumber
    int pos_num = get_pos_num(msg->pose.position.x);
    //first entry
    if(chips_green_pos_history.size()==0){
      chips_green_pos_history.push_back(pos_num);
    }
    //if we have the same pos
    if(chips_green_pos_history[chips_green_pos_history.size()-1]==pos_num){
      //ROS_INFO("same");
    }
    //new pose
    if(chips_green_pos_history[chips_green_pos_history.size()-1]!=pos_num){
      chips_green_pos_history.push_back(pos_num);
      ROS_DEBUG_NAMED("simtrack_gui", "new move chips_green %i",pos_num);
    }
    int temp=chips_green_pos_history.size();
    //check if we have a movement
    //check if there is enough data to do movment check
    if(chips_green_pos_history.size()<consec_needed+1){
      //ROS_INFO("to fee pos");
      return;
    }
    //look for movement to the right
    int count=0;
    for(int i=0;i<chips_green_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(chips_green_pos_history[i+j]+1==chips_green_pos_history[i+j+1]){
          count++;
        }
      }
        //do we move to the right?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to the right detected");
        cart_content.push_back(chips_green_name);
        cart_costs.push_back(chips_green_cost);
        fill_cart();
        //delete history as we have the object
        chips_green_pos_history.clear();
        return;
      }
      count=0;
    }
    
    //look for movement to the left
    count=0;
    for(int i=0;i<chips_green_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(chips_green_pos_history[i+j]-1==chips_green_pos_history[i+j+1]){
          count++;
        }
      }
      //do we move to the left?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to left detected");
        //find pos of entry
        int del_pos=-1;
        for(int i=0;i<cart_content.size();i++){
          if(cart_content[i]==chips_green_name)
            del_pos=i;
        }
        if(del_pos==-1){
          chips_green_pos_history.clear();
          return;
        }
        else{

          
          cart_content.erase(cart_content.begin() + del_pos);
          cart_costs.erase(cart_costs.begin() + del_pos);
          fill_cart();
          //delete history as we have the object
          chips_green_pos_history.clear();
          return;
        } 
        count=0;
      }         
    }
  }


   void SimtrackPanel::coke_can_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    ROS_DEBUG_THROTTLE(5,"Pose Callback");
    //abourt if to close
    if(msg->pose.position.z<0.3)
      return;
    //we get callback when we track the thing ... hopefully ... 
    //=> initilize object when callback comes and reset if no callback in t secs
    //check if the thing is on screen
    if(ros::Time::now()>=coke_can_off_screen_time+new_object_time){
      //it was gone to long
      coke_can_pos_history.clear();
      ROS_DEBUG_NAMED("simtrack_gui", "New object detected: coke_can_");
    }
    //update time
    coke_can_off_screen_time=ros::Time::now();
    //get the movenumber
    int pos_num = get_pos_num(msg->pose.position.x);
    //first entry
    if(coke_can_pos_history.size()==0){
      coke_can_pos_history.push_back(pos_num);
    }
    //if we have the same pos
    if(coke_can_pos_history[coke_can_pos_history.size()-1]==pos_num){
      //ROS_INFO("same");
    }
    //new pose
    if(coke_can_pos_history[coke_can_pos_history.size()-1]!=pos_num){
      coke_can_pos_history.push_back(pos_num);
      ROS_DEBUG_NAMED("simtrack_gui", "new move coke_can_ %i",pos_num);
    }
    int temp=coke_can_pos_history.size();
    //check if we have a movement
    //check if there is enough data to do movment check
    if(coke_can_pos_history.size()<consec_needed+1){
      //ROS_INFO("to fee pos");
      return;
    }
    //look for movement to the right
    int count=0;
    for(int i=0;i<coke_can_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(coke_can_pos_history[i+j]+1==coke_can_pos_history[i+j+1]){
          count++;
        }
      }
        //do we move to the right?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to the right detected");
        cart_content.push_back(coke_can_name);
        cart_costs.push_back(coke_can_cost);
        fill_cart();
        //delete history as we have the object
        coke_can_pos_history.clear();
        return;
      }
      count=0;
    }
    
    //look for movement to the left
    count=0;
    for(int i=0;i<coke_can_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(coke_can_pos_history[i+j]-1==coke_can_pos_history[i+j+1]){
          count++;
        }
      }
      //do we move to the left?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to left detected");
        //find pos of entry
        int del_pos=-1;
        for(int i=0;i<cart_content.size();i++){
          if(cart_content[i]==coke_can_name)
            del_pos=i;
        }
        if(del_pos==-1){
          coke_can_pos_history.clear();
          return;
        }
        else{

          
          cart_content.erase(cart_content.begin() + del_pos);
          cart_costs.erase(cart_costs.begin() + del_pos);
          fill_cart();
          //delete history as we have the object
          coke_can_pos_history.clear();
          return;
        } 
        count=0;
      }         
    }
  }


   void SimtrackPanel::m_dew_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    ROS_DEBUG_THROTTLE(5,"Pose Callback");
    //abourt if to close
    if(msg->pose.position.z<0.3)
      return;
    //we get callback when we track the thing ... hopefully ... 
    //=> initilize object when callback comes and reset if no callback in t secs
    //check if the thing is on screen
    if(ros::Time::now()>=m_dew_off_screen_time+new_object_time){
      //it was gone to long
      m_dew_pos_history.clear();
      ROS_DEBUG_NAMED("simtrack_gui", "New object detected: m_dew");
    }
    //update time
    m_dew_off_screen_time=ros::Time::now();
    //get the movenumber
    int pos_num = get_pos_num(msg->pose.position.x);
    //first entry
    if(m_dew_pos_history.size()==0){
      m_dew_pos_history.push_back(pos_num);
    }
    //if we have the same pos
    if(m_dew_pos_history[m_dew_pos_history.size()-1]==pos_num){
      //ROS_INFO("same");
    }
    //new pose
    if(m_dew_pos_history[m_dew_pos_history.size()-1]!=pos_num){
      m_dew_pos_history.push_back(pos_num);
      ROS_DEBUG_NAMED("simtrack_gui", "new move m_dew %i",pos_num);
    }
    int temp=m_dew_pos_history.size();
    //check if we have a movement
    //check if there is enough data to do movment check
    if(m_dew_pos_history.size()<consec_needed+1){
      //ROS_INFO("to fee pos");
      return;
    }
    //look for movement to the right
    int count=0;
    for(int i=0;i<m_dew_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(m_dew_pos_history[i+j]+1==m_dew_pos_history[i+j+1]){
          count++;
        }
      }
        //do we move to the right?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to the right detected");
        cart_content.push_back(m_dew_name);
        cart_costs.push_back(m_dew_cost);
        fill_cart();
        //delete history as we have the object
        m_dew_pos_history.clear();
        return;
      }
      count=0;
    }
    
    //look for movement to the left
    count=0;
    for(int i=0;i<m_dew_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(m_dew_pos_history[i+j]-1==m_dew_pos_history[i+j+1]){
          count++;
        }
      }
      //do we move to the left?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to left detected");
        //find pos of entry
        int del_pos=-1;
        for(int i=0;i<cart_content.size();i++){
          if(cart_content[i]==m_dew_name)
            del_pos=i;
        }
        if(del_pos==-1){
          m_dew_pos_history.clear();
          return;
        }
        else{

          
          cart_content.erase(cart_content.begin() + del_pos);
          cart_costs.erase(cart_costs.begin() + del_pos);
          fill_cart();
          //delete history as we have the object
          m_dew_pos_history.clear();
          return;
        } 
        count=0;
      }         
    }
  }



   void SimtrackPanel::can_corn_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    ROS_DEBUG_THROTTLE(5,"Pose Callback");
    //abourt if to close
    if(msg->pose.position.z<0.3)
      return;
    //we get callback when we track the thing ... hopefully ... 
    //=> initilize object when callback comes and reset if no callback in t secs
    //check if the thing is on screen
    if(ros::Time::now()>=can_corn_off_screen_time+new_object_time){
      //it was gone to long
      can_corn_pos_history.clear();
      ROS_DEBUG_NAMED("simtrack_gui", "New object detected: can_corn");
    }
    //update time
    can_corn_off_screen_time=ros::Time::now();
    //get the movenumber
    int pos_num = get_pos_num(msg->pose.position.x);
    //first entry
    if(can_corn_pos_history.size()==0){
      can_corn_pos_history.push_back(pos_num);
    }
    //if we have the same pos
    if(can_corn_pos_history[can_corn_pos_history.size()-1]==pos_num){
      //ROS_INFO("same");
    }
    //new pose
    if(can_corn_pos_history[can_corn_pos_history.size()-1]!=pos_num){
      can_corn_pos_history.push_back(pos_num);
      ROS_DEBUG_NAMED("simtrack_gui", "new move can_corn %i",pos_num);
    }
    int temp=can_corn_pos_history.size();
    //check if we have a movement
    //check if there is enough data to do movment check
    if(can_corn_pos_history.size()<consec_needed+1){
      //ROS_INFO("to fee pos");
      return;
    }
    //look for movement to the right
    int count=0;
    for(int i=0;i<can_corn_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(can_corn_pos_history[i+j]+1==can_corn_pos_history[i+j+1]){
          count++;
        }
      }
        //do we move to the right?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to the right detected");
        cart_content.push_back(can_corn_name);
        cart_costs.push_back(can_corn_cost);
        fill_cart();
        //delete history as we have the object
        can_corn_pos_history.clear();
        return;
      }
      count=0;
    }
    
    //look for movement to the left
    count=0;
    for(int i=0;i<can_corn_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(can_corn_pos_history[i+j]-1==can_corn_pos_history[i+j+1]){
          count++;
        }
      }
      //do we move to the left?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to left detected");
        //find pos of entry
        int del_pos=-1;
        for(int i=0;i<cart_content.size();i++){
          if(cart_content[i]==can_corn_name)
            del_pos=i;
        }
        if(del_pos==-1){
          can_corn_pos_history.clear();
          return;
        }
        else{

          
          cart_content.erase(cart_content.begin() + del_pos);
          cart_costs.erase(cart_costs.begin() + del_pos);
          fill_cart();
          //delete history as we have the object
          can_corn_pos_history.clear();
          return;
        } 
        count=0;
      }         
    }
  }


   void SimtrackPanel::can_borsch_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    ROS_DEBUG_THROTTLE(5,"Pose Callback");
    //abourt if to close
    if(msg->pose.position.z<0.3)
      return;
    //we get callback when we track the thing ... hopefully ... 
    //=> initilize object when callback comes and reset if no callback in t secs
    //check if the thing is on screen
    if(ros::Time::now()>=can_borsch_off_screen_time+new_object_time){
      //it was gone to long
      can_borsch_pos_history.clear();
      ROS_DEBUG_NAMED("simtrack_gui", "New object detected: can_borsch");
    }
    //update time
    can_borsch_off_screen_time=ros::Time::now();
    //get the movenumber
    int pos_num = get_pos_num(msg->pose.position.x);
    //first entry
    if(can_borsch_pos_history.size()==0){
      can_borsch_pos_history.push_back(pos_num);
    }
    //if we have the same pos
    if(can_borsch_pos_history[can_borsch_pos_history.size()-1]==pos_num){
      //ROS_INFO("same");
    }
    //new pose
    if(can_borsch_pos_history[can_borsch_pos_history.size()-1]!=pos_num){
      can_borsch_pos_history.push_back(pos_num);
      ROS_DEBUG_NAMED("simtrack_gui", "new move can_borsch %i",pos_num);
    }
    int temp=can_borsch_pos_history.size();
    //check if we have a movement
    //check if there is enough data to do movment check
    if(can_borsch_pos_history.size()<consec_needed+1){
      //ROS_INFO("to fee pos");
      return;
    }
    //look for movement to the right
    int count=0;
    for(int i=0;i<can_borsch_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(can_borsch_pos_history[i+j]+1==can_borsch_pos_history[i+j+1]){
          count++;
        }
      }
        //do we move to the right?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to the right detected");
        cart_content.push_back(can_borsch_name);
        cart_costs.push_back(can_borsch_cost);
        fill_cart();
        //delete history as we have the object
        can_borsch_pos_history.clear();
        return;
      }
      count=0;
    }
    
    //look for movement to the left
    count=0;
    for(int i=0;i<can_borsch_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(can_borsch_pos_history[i+j]-1==can_borsch_pos_history[i+j+1]){
          count++;
        }
      }
      //do we move to the left?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to left detected");
        //find pos of entry
        int del_pos=-1;
        for(int i=0;i<cart_content.size();i++){
          if(cart_content[i]==can_borsch_name)
            del_pos=i;
        }
        if(del_pos==-1){
          can_borsch_pos_history.clear();
          return;
        }
        else{

          
          cart_content.erase(cart_content.begin() + del_pos);
          cart_costs.erase(cart_costs.begin() + del_pos);
          fill_cart();
          //delete history as we have the object
          can_borsch_pos_history.clear();
          return;
        } 
        count=0;
      }         
    }
  }


   void SimtrackPanel::cornflaks_code_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    ROS_DEBUG_THROTTLE(5,"Pose Callback");
    //abourt if to close
    if(msg->pose.position.z<0.3)
      return;
    //we get callback when we track the thing ... hopefully ... 
    //=> initilize object when callback comes and reset if no callback in t secs
    //check if the thing is on screen
    if(ros::Time::now()>=cornflaks_off_screen_time+new_object_time){
      //it was gone to long
      cornflaks_pos_history.clear();
      ROS_DEBUG_NAMED("simtrack_gui", "New object detected: cornflaks");
    }
    //update time
    cornflaks_off_screen_time=ros::Time::now();
    //get the movenumber
    int pos_num = get_pos_num(msg->pose.position.x);
    //first entry
    if(cornflaks_pos_history.size()==0){
      cornflaks_pos_history.push_back(pos_num);
    }
    //if we have the same pos
    if(cornflaks_pos_history[cornflaks_pos_history.size()-1]==pos_num){
      //ROS_INFO("same");
    }
    //new pose
    if(cornflaks_pos_history[cornflaks_pos_history.size()-1]!=pos_num){
      cornflaks_pos_history.push_back(pos_num);
      ROS_DEBUG_NAMED("simtrack_gui", "new move cornflaks %i",pos_num);
    }
    int temp=cornflaks_pos_history.size();
    //check if we have a movement
    //check if there is enough data to do movment check
    if(cornflaks_pos_history.size()<consec_needed+1){
      //ROS_INFO("to fee pos");
      return;
    }
    //look for movement to the right
    int count=0;
    for(int i=0;i<cornflaks_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(cornflaks_pos_history[i+j]+1==cornflaks_pos_history[i+j+1]){
          count++;
        }
      }
        //do we move to the right?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to the right detected");
        cart_content.push_back(cornflaks_name);
        cart_costs.push_back(cornflaks_cost);
        fill_cart();
        //delete history as we have the object
        cornflaks_pos_history.clear();
        return;
      }
      count=0;
    }
    
    //look for movement to the left
    count=0;
    for(int i=0;i<cornflaks_pos_history.size()-consec_needed-1;i++){
      for(int j=0;j<consec_needed;j++){
        if(cornflaks_pos_history[i+j]-1==cornflaks_pos_history[i+j+1]){
          count++;
        }
      }
      //do we move to the left?
      if(count>=consec_needed){
        ROS_DEBUG_NAMED("simtrack_gui", "move to left detected");
        //find pos of entry
        int del_pos=-1;
        for(int i=0;i<cart_content.size();i++){
          if(cart_content[i]==cornflaks_name)
            del_pos=i;
        }
        if(del_pos==-1){
          cornflaks_pos_history.clear();
          return;
        }
        else{

          
          cart_content.erase(cart_content.begin() + del_pos);
          cart_costs.erase(cart_costs.begin() + del_pos);
          fill_cart();
          //delete history as we have the object
          cornflaks_pos_history.clear();
          return;
        } 
        count=0;
      }         
    }
  }




  int SimtrackPanel::get_pos_num(double pos){
    int result =pos/size_of_fields;
    return result;
  }


  void SimtrackPanel::fill_cart(){
    //fill the cart given the 2 vectors with name and price
    
    //empty everything
    lbl_item_01_->setText("");
    lbl_cost_01_->setText("");
    lbl_item_02_->setText("");
    lbl_cost_02_->setText("");
    lbl_item_03_->setText("");
    lbl_cost_03_->setText("");
    lbl_item_04_->setText("");
    lbl_cost_04_->setText("");
    lbl_item_05_->setText("");
    lbl_cost_05_->setText("");
    lbl_item_06_->setText("");
    lbl_cost_06_->setText("");
    lbl_item_07_->setText("");
    lbl_cost_07_->setText("");
    lbl_item_08_->setText("");
    lbl_cost_08_->setText("");
    lbl_item_09_->setText("");
    lbl_cost_09_->setText("");
    lbl_item_10_->setText("");
    lbl_cost_10_->setText("");

      if(cart_content.size()==0){
        lbl_total_amount_->setText(" ");
        return;
      }
        

    for(int i=0;i<cart_content.size();i++){
      switch (i){
        case 0: lbl_item_01_->setText((cart_content[i]));
                lbl_cost_01_->setText(QString::number(cart_costs[i]) + " HKD");
                break;
        case 1: lbl_item_02_->setText((cart_content[i]));
                lbl_cost_02_->setText(QString::number(cart_costs[i]) + " HKD");
                break;
        case 2: lbl_item_03_->setText((cart_content[i]));
                lbl_cost_03_->setText(QString::number(cart_costs[i]) + " HKD");
                break;
        case 3: lbl_item_04_->setText((cart_content[i]));
                lbl_cost_04_->setText(QString::number(cart_costs[i]) + " HKD");
                break;
        case 4: lbl_item_05_->setText((cart_content[i]));
                lbl_cost_05_->setText(QString::number(cart_costs[i]) + " HKD");
                break;
        case 5: lbl_item_06_->setText((cart_content[i]));
                lbl_cost_06_->setText(QString::number(cart_costs[i]) + " HKD");
                break;
        case 6: lbl_item_07_->setText((cart_content[i]));
                lbl_cost_07_->setText(QString::number(cart_costs[i]) + " HKD");
                break;
        case 7: lbl_item_08_->setText((cart_content[i]));
                lbl_cost_08_->setText(QString::number(cart_costs[i]) + " HKD");
                break;
        case 8: lbl_item_09_->setText((cart_content[i]));
                lbl_cost_09_->setText(QString::number(cart_costs[i]) + " HKD");
                break;
        case 9: lbl_item_10_->setText((cart_content[i]));
                lbl_cost_10_->setText(QString::number(cart_costs[i]) + " HKD");
                break;        
      }
    }
    // total ammount
    double total_amount=0;
    for(int i=0;i<cart_costs.size();i++)
      total_amount+=cart_costs[i];
    lbl_total_amount_->setText(QString::number(total_amount) + " HKD");

  }

//receive status updates from game_master
void SimtrackPanel::clear_cart()
  {
    ROS_DEBUG_NAMED("Simtrack_gui","Clear cart");
    cart_content.clear();
    cart_costs.clear();
    fill_cart();

  }

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void SimtrackPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}
// Load all configuration data for this panel from the given Config object.
void SimtrackPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}
}  // end namespace 

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(simtrack_gui::SimtrackPanel, rviz::Panel)