//************************************************
//**********Baxter 2017 Tic-Tac-Toe***************
//*******Nadine Drollinger & Michael Welle********
//************************************************
//*******Camera node - board_detector*************
//************************************************

//************************************************
//Description: detects game board from the cut out 
// real board.
//************************************************

//Includes
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/video/background_segm.hpp"
#include "opencv2/video/tracking.hpp"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

//Namespace
using namespace cv;
using namespace std;

//Class board_detector
class board_detector
{
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_cutout_;
  
  public:
    bool debug_flag;
    int rows;
    int cols;
    int** gameboard;
    int r_threshold;
    int b_threshold;
	//opencv help
	vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //opencv images
    Mat org, subImage, input_grey;
	Mat filter_image_red, filter_image_blue;
	Mat canny_output, drawing, countourtest;

  board_detector() : it_(n_), debug_flag(true), rows(7), cols(7), r_threshold(100),b_threshold(100) 
  {
  	// Subscrive and publisher
    image_sub_cutout_ = it_.subscribe("/TTTgame/cut_board", 1, &board_detector::imageCb, this);
    
    //init game board
    gameboard = new int*[rows];
	for(int i = 0; i < rows; ++i)
    	gameboard[i] = new int[cols];
   
    //debug
    if(debug_flag){
      namedWindow("Input_cutout", CV_WINDOW_AUTOSIZE);
      namedWindow("Contours", CV_WINDOW_AUTOSIZE );
      namedWindow( "gameboard contour", CV_WINDOW_AUTOSIZE );
      namedWindow("Gameboard", CV_WINDOW_AUTOSIZE );
    }

    
  }

  ~board_detector()
    {
      if(debug_flag){
        destroyWindow("Contours");
        destroyWindow("Input_cutout"); 
        destroyWindow("gameboard contour");
        destroyWindow("Gameboard");
      }    
    }

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

      if(debug_flag){
      	imshow( "Input_cutout", org );
      }

      //check if there is an image ...
      if(org.size().width==0 || org.size().height==0)
      	return;
      
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
        waitKey(10);
        //namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
        imshow( "Contours", drawing );
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
	            //bounding_rect = boundingRect( contours[i] ); // Find the bounding rectangle for biggest contour
	        }
	    }

	  //output for sclicing up the thing
      Mat warpedCard(400, 400, CV_8UC3);
      //check that the largest area is at least half of the image
	  if(largest_area>org.rows*org.cols/2)
		{
		  if(debug_flag)
		  {
		    //draw onlz bigges countur
		    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		    drawContours( countourtest, contours, largest_contour_index, color);
		    imshow( "gameboard contour", countourtest );
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
		    imshow("Ctr",mc);
		  }
		  
          //target points for homogentranform
		  vector<Point2f> dest;
		  dest.push_back(Point2f(warpedCard.cols, 0.0));
		  dest.push_back(Point2f(0,0));
		  dest.push_back(Point2f(0, warpedCard.rows));
		  dest.push_back(Point2f(warpedCard.cols, warpedCard.rows));

          //inputpoints
          //make sure approx and dest are in the right order ALWAZS
		  vector<Point2f> inpoint;
		  for(int j=0; j<corners.size();j++){
		  	//check for first point
		  	if(corners[j].x<warpedCard.cols/2 && corners[j].y<warpedCard.rows/2)
		  	  inpoint.push_back(corners[j]);
		  	
		  	//check for second point
		  	if(corners[j].x>warpedCard.cols/2 && corners[j].y<warpedCard.rows/2)
		  	  inpoint.push_back(corners[j]);
		  	
		  	//check for 3 point
		  	if(corners[j].x>warpedCard.cols/2 && corners[j].y>warpedCard.rows/2)
		  	  inpoint.push_back(corners[j]);		  	
		  	//check for 4 point
		  	if(corners[j].x<warpedCard.cols/2 && corners[j].y>warpedCard.rows/2)
		  	  inpoint.push_back(corners[j]);
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
            
            if(debug_flag){
            	imshow("Gameboard", warpedCard);
            }

            //slize it up!!!
	        // parameters
		    int rowcount=0;
		    int colcount=0;
		    int image_width=warpedCard.size().width;
		    int image_height=warpedCard.size().height;
		    //defult rect
		    Rect Rec(0, 0, 10, 10);

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
				namedWindow("show sub", WINDOW_AUTOSIZE);
				imshow("Step show sub", subImage);
				waitKey(100);
			  }   

			  //get mean color of ROI
			  Scalar meancolor = cv::mean(subImage);
			  if(debug_flag){
			    cout << "row: " << rowcount << " col: " << colcount << " R: " << meancolor[0] << " G: " << meancolor [1] << " B: " << meancolor[2] << endl;
			  }
			  			    
			  // what piece is it?
			  if(meancolor[0]<r_threshold && meancolor[2]>b_threshold ){
			    //ROS_INFO_STREAM( " Red-piece detected" );
			    gameboard[rowcount][colcount]=2;
			  }
			  if(meancolor[0]>r_threshold && meancolor[2]<b_threshold ){
			    //ROS_INFO_STREAM( " Blue-piece detected" );	
				gameboard[rowcount][colcount]=1;
			  }
			  //if less than 1% is red and les then 1% blue -> its no-piece	    
			  if(meancolor[0]>r_threshold && meancolor[2]>b_threshold ){
			    //ROS_INFO_STREAM( " No-piece detected" );	
				gameboard[rowcount][colcount]=0;
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

			 waitKey(10);
				
		    }

		    //Print gameboard
		    cout << "*********************************" << endl;
		    for(int i=0;i<rows;i++){
			  for(int j=0;j<cols;j++){
				cout << "|" << gameboard[i][j];
			  }
			  cout << "|" << endl;
			  cout << "_______________" << endl;		
		    }
		    cout << "*********************************" << endl;
		  
	    }
    }
    


      
      
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "board_detector");
  board_detector bd;
  ROS_INFO("board detector initilized");
  ros::spin();
  return 0;
}

