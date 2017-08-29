//************************************************
//**********Baxter 2017 Tic-Tac-Toe***************
//*******Nadine Drollinger & Michael Welle********
//************************************************
//*******Camera node - camera ********************
//************************************************

//************************************************
//Description: detects game board from the cut out 
// real board with the help of the outer boarder.
// detects game pieces and fills in gameboard array
// with a diff threshold (diffence between blue 
// and red)
//************************************************

//Includes
#include <ros/ros.h>
//actionlib
#include <actionlib/server/simple_action_server.h>
#include <camera/camera_game_masterAction.h>
//opencv
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/video/background_segm.hpp"
#include "opencv2/video/tracking.hpp"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
//others
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"

//Namespace
using namespace cv;
using namespace std;

class camera_boss
{
protected:
  ros::NodeHandle nh_;
  //opencv image transport
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_cutout_;
  //actionlib
  actionlib::SimpleActionServer<camera::camera_game_masterAction> as_camera; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  camera::camera_game_masterFeedback feedback_camera; // create messages that are used to published feedback
  camera::camera_game_masterResult result_camera;    // create messages that are used to published result
  image_transport::Publisher image_pub_gameboard_;

public:
  //debug
  bool debug_flag;
  //gamboard
  bool cut_board_ok;
  int rows;
  int cols;
  std::vector<int> gameboard; 
  int diff_threshold;
  int player;
  //opencv help
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  //opencv images
  Mat org, subImage, input_grey;
  Mat filter_image_red, filter_image_blue;
  Mat canny_output, drawing, countourtest;
  
  //NOTE:Debug camera only works if the debug imshow windows are closed before the next request ... it crasehes otherwise

  //constructor
  camera_boss(std::string name) :
    as_camera(nh_, name, boost::bind(&camera_boss::camera_start_command, this, _1), false),
    action_name_(name),it_(nh_), debug_flag(false), rows(6), cols(6),
    diff_threshold(25),cut_board_ok(false)
  {
    as_camera.start();
    // Subscrive and publisher
    image_sub_cutout_ = it_.subscribe("/TTTgame/cut_board", 1, &camera_boss::imageCb, this);
    image_pub_gameboard_ = it_.advertise("/TTTgame/only_gameboard", 1);
    
    //debug
        if(debug_flag){
      namedWindow("Input_cutout", CV_WINDOW_AUTOSIZE);
    }
  }

  //deconstructor
  ~camera_boss(void)
  {
    if(debug_flag){
      destroyWindow("Input_cutout"); 
    }
  }

  //image callback from the cut out board
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
      //read in image from subscribed topic and transform it to opencv
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      // process the image
      org = cv_ptr->image;
      //debug
      if(debug_flag){
        waitKey(1);
        imshow( "Input_cutout", org );
        waitKey(1);
      }
      //check if there is an image ...
      if(org.size().width==0 || org.size().height==0){
        cut_board_ok=false;
      }   
      else{
        cut_board_ok=true;
      }     
    }

  //callback stgarte from actionlib (send from game_maste)
  void camera_start_command(const camera::camera_game_masterGoalConstPtr &goal)
  {

    ROS_INFO("Preparing camera update");
    player=goal->update;
    // helper variables
    ros::Rate r(1);
    bool success = false;

    //check if cut board is ok. If not send feedback
    if(!cut_board_ok){
      ROS_INFO("Problems with cut_board");
      feedback_camera.progress=-1;      
      as_camera.publishFeedback(feedback_camera);
      return;
    }

    /*
    // this breaks the node after one run ...
    if(debug_flag){
      
      namedWindow("Contours", CV_WINDOW_AUTOSIZE );
      namedWindow("gameboard contour", CV_WINDOW_AUTOSIZE );
      namedWindow("Gameboard", CV_WINDOW_AUTOSIZE );
      namedWindow("show sub", WINDOW_AUTOSIZE);
      namedWindow("Ctr", CV_WINDOW_AUTOSIZE );
      
    }
    */
  
    // process the cut out board and detect the pieces***
    // feddback
    feedback_camera.progress=0; // progress in %    
    as_camera.publishFeedback(feedback_camera);

    //convert to gray scale
    cvtColor(org, input_grey, CV_BGR2GRAY);
    // find contur in the image  
    int thresh = 100;
    int max_thresh = 255;
    RNG rng(12345);
    // blur the image with gausian
    blur(input_grey, input_grey, Size(3, 3));
    // Detect edges using canny
    Canny( org, canny_output, thresh, thresh*2, 3 );
    // Find contours
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
      
    if(debug_flag){
      // Draw contours
      drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
      countourtest;
      drawing.copyTo(countourtest);
        
      for( int i = 0; i< contours.size(); i++ )
      {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
      }
      // Show in a window
      waitKey(1);
      imshow( "Contours", drawing );
      waitKey(1);
    }

    //find lagrest countur (gameboard boundingbox)
    int largest_area=0;
    int largest_contour_index=0;

    // get bigges conure
    for( size_t i = 0; i< contours.size(); i++ ) // iterate through each contour.
      {
        double area = contourArea( contours[i] );  //  Find the area of contour
        if( area > largest_area )
        {
          largest_area = area;
          largest_contour_index = i;               //Store the index of largest contour
        }
      }

    //output for sclicing up the thing
    Mat warpedCard(400, 400, CV_8UC3);
    //check that the largest area is at least half of the image
    if(largest_area>org.rows*org.cols/2.5)
    {
      if(debug_flag)
      {
        //draw onlz bigges countur
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( countourtest, contours, largest_contour_index, color);
        waitKey(1);
        imshow( "gameboard contour", countourtest );
        waitKey(1);
      }

      vector<Point> corners;
      double d=0;
      do
      {
        d=d+1;
        approxPolyDP(contours[largest_contour_index],corners,d,true);
      }
      while (corners.size()>4);
        
      contours.push_back(corners);
        
      if(debug_flag){
        Mat mc=org;
        drawContours(mc,contours,contours.size()-1,Scalar(0,0,255),1);
        waitKey(1);
        imshow("Ctr",mc);
        waitKey(1);
        cout << "Ctr" << endl;
      }
  
      //target points for homogentranform
      vector<Point2f> dest;
      
      
      dest.push_back(Point2f(0,0)); 
      dest.push_back(Point2f(warpedCard.cols, 0.0));    
      dest.push_back(Point2f(warpedCard.cols, warpedCard.rows));  
      dest.push_back(Point2f(0, warpedCard.rows));  
       

      //inputpoints
      //make sure approx and dest are in the right order ALWAZS
      vector<Point2f> inpoint;
      cout << " org cols/2: " << org.cols/2 << " org rows/2  " << org.rows/2 << endl;
      for(int j=0; j<corners.size();j++){
        //check for first point
        if(corners[j].x<org.cols/2 && corners[j].y<org.rows/2){
          inpoint.push_back(corners[j]);
          //cout << "detected corner 1 (0,0) : x: " << corners[j].x << " y: " << corners[j].y << endl;
        }
      }
      for(int j=0; j<corners.size();j++){
        //check for second point
        if(corners[j].x>org.cols/2 && corners[j].y<org.rows/2){
          inpoint.push_back(corners[j]);
          //cout << "detected corner 2 (0,max) : x: " << corners[j].x << " y: " << corners[j].y << endl;
        }
      }
      for(int j=0; j<corners.size();j++){
        //check for 3 point
        if(corners[j].x>org.cols/2 && corners[j].y>org.rows/2){
          inpoint.push_back(corners[j]);
          //cout << "detected corner 3 (max,max) : x: " << corners[j].x << " y: " << corners[j].y << endl;
        }
      }
      for(int j=0; j<corners.size();j++){
        //check for 4 point
        if(corners[j].x<org.cols/2 && corners[j].y>org.rows/2){
          inpoint.push_back(corners[j]);
          //cout << "detected corner 4 (max,0) : x: " << corners[j].x << " y: " << corners[j].y << endl;
        }
      }

      if(debug_flag){
        cout << "aprox size : " << inpoint.size() << endl;
        cout << "p1: " << inpoint[0] << " p2: " << inpoint[1] << " p3: " << inpoint[2] << " p4: " << inpoint[3] << endl;
      }
        
      if (corners.size() == 4)
        {
          Mat homography = findHomography(inpoint, dest);
          warpPerspective(org, warpedCard, homography, Size(warpedCard.cols, warpedCard.rows));
        }
        else{
          //we fail
          result_camera.gameboard = gameboard;
          result_camera.fail =2;
          as_camera.setSucceeded(result_camera);
          ROS_INFO("Cornersize wrong");
          return;
        }   
          
        if(debug_flag){
          waitKey(1);
          imshow("Gameboard", warpedCard);
          waitKey(1);
        }

        //slize it up!!!
        int rowcount=0;
        int colcount=0;
        int image_width=warpedCard.size().width;
        int image_height=warpedCard.size().height;
        //defult rect
        Rect Rec(0, 0, 10, 10);
        //clear the gameboard
        gameboard.clear();

        //Loop through all ROI and se if its O or X
        for(int i=0;i<cols*rows;i++){
          //ROI
          Rec.x=image_width/cols*colcount;
          Rec.y=image_height/rows*rowcount;
          Rec.width=image_width/cols;
          Rec.height=image_height/rows;
          
          //copy to new sub image 
          subImage = warpedCard(Rec).clone();
          waitKey(10);

          //debug subimages  
          if(debug_flag){
            waitKey(1);
            imshow("Step show sub", subImage);
            waitKey(100);
          }   

          //get mean color of ROI
          Scalar meancolor = cv::mean(subImage);
          
          if(debug_flag){
            cout << "row: " << rowcount << " col: " << colcount << " R: " << meancolor[0] << " G: " << meancolor [1] << " B: " << meancolor[2] << endl;
            cout << "diff red : " << meancolor[2] - meancolor[0] <<  endl;
            cout << "diff blue: " << meancolor[0] - meancolor[2] <<  endl;
          }

          // difference aproch to detect pieces
          if(meancolor[2] - meancolor[0] > diff_threshold ){
            if(debug_flag)
              ROS_INFO_STREAM( " Red-piece detected" );
            gameboard.push_back(2);
          }
          if(meancolor[0] - meancolor[2] > diff_threshold){
            if(debug_flag)
              ROS_INFO_STREAM( " Blue-piece detected" );  
            gameboard.push_back(1);
          }
          if(meancolor[2] - meancolor[0] < diff_threshold && meancolor[0] - meancolor[2] < diff_threshold){
            if(debug_flag)
              ROS_INFO_STREAM( " No-piece detected" );  
            gameboard.push_back(0);
          }

          //keep track of ROI
          colcount+=1;
          if (colcount>cols-1){
            colcount=0;
            rowcount+=1;
            if (rowcount>rows-1){
              rowcount=0;
            }
          }
          
        }

        // feddback
        feedback_camera.progress=100; // progress in %    
        as_camera.publishFeedback(feedback_camera);
        //Camera did it
        success=true;
        
        if(debug_flag){
          //Print gameboard
          cout << "*********************************" << endl;
          for(int i=0;i<rows;i++){
          for(int j=0;j<cols;j++){
          cout << "|" << gameboard[j+rows*i];
          }
          cout << "|" << endl;
          cout << "_______________" << endl;    
          }
          cout << "*********************************" << endl;
        }
        

      }
      else{
        // feddback
        ROS_INFO(" Found Contour to small");
        feedback_camera.progress=-2; // progress in %    
        as_camera.publishFeedback(feedback_camera);
      }

  	  if(success)
      {
      	ROS_INFO("camera update done");
        gameboard.push_back(player);
        result_camera.gameboard = gameboard;
        result_camera.fail =1;
        // set the action state to succeeded
        as_camera.setSucceeded(result_camera);
        //publish the image in ros
      cv_bridge::CvImage out_msg;
      out_msg.header.frame_id   = "/world";//cv_ptr->header; // Same timestamp and tf frame as input image
      out_msg.header.stamp =ros::Time::now(); // new timestamp
      out_msg.encoding = sensor_msgs::image_encodings::BGR8; // encoding, might need to try some diffrent ones
      out_msg.image    = warpedCard; 
      image_pub_gameboard_.publish(out_msg.toImageMsg()); //transfer to Ros image message  

      }
      else{
        //We failed
        result_camera.gameboard = gameboard;
        result_camera.fail =2;
        as_camera.setSucceeded(result_camera);
        ROS_INFO("send feedback");
      }

      /*
      // doesent work ...
     if(debug_flag){
      
        destroyWindow("Contours");        
        destroyWindow("gameboard contour");
        destroyWindow("Gameboard");
        destroyWindow("show sub");
        destroyWindow("Ctr");
        
      } 
      */ 
    }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_game_master");
  ROS_INFO("Start camera node");
  //start action server
  camera_boss ab("camera_game_master");
 while(ros::ok()){
  ros::spinOnce();
  //ROS_INFO("Still alive");
  ros::Duration(1.0).sleep();
 }
  
   

  return 0;
}
