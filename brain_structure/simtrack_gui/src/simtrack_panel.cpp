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
SimtrackPanel::SimtrackPanel(QWidget* parent) : rviz::Panel(parent),consec_needed(5),size_of_fields(0.077),new_object_time(5.0),fuerte_name("Fuerte turtel"),fuerte_cost(8.77),milk_name("Skimmed milk"),milk_cost(29.85)
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
      ROS_DEBUG_NAMED("simtrack_gui", "New object detected: fuerte");
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
      ROS_DEBUG_NAMED("simtrack_gui", "new move fuerte %i",pos_num);
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