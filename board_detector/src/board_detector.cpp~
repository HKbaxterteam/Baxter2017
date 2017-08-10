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
    int filter_red_r_low;
	int filter_red_g_low;
	int filter_red_b_low;
	int filter_red_r_high;
	int filter_red_g_high;
	int filter_red_b_high;
	int filter_blue_r_low;
	int filter_blue_g_low;
	int filter_blue_b_low;
	int filter_blue_r_high;
	int filter_blue_g_high;
	int filter_blue_b_high;
    //opencv images
    Mat org, subImage;
	Mat filter_image_red, filter_image_blue;

  board_detector() : it_(n_), debug_flag(true), rows(7), cols(7), filter_red_r_low(0), filter_red_g_low(0), filter_red_b_low(0), filter_red_r_high(255), filter_red_g_high(25), filter_red_b_high(25), filter_blue_r_low(0), filter_blue_g_low(0), filter_blue_b_low(50), filter_blue_r_high(25), filter_blue_g_high(25), filter_blue_b_high(255) 
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
      namedWindow("ROI-section", CV_WINDOW_AUTOSIZE);
      namedWindow("ROI-section-filtered", CV_WINDOW_AUTOSIZE);
    }
  }

  ~board_detector()
    {
      if(debug_flag){
        destroyWindow("Input_cutout");
        destroyWindow("ROI-section"); 
        destroyWindow("ROI-section-filtered");
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

      //check if there is an image ...
      if(org.size().width==0 || org.size().height==0)
      	return;
      
      // parameters
	  int rowcount=0;
	  int colcount=0;
	  int image_width=org.size().width;
	  int image_height=org.size().height;
	  //defult rect
	  Rect Rec(0, 0, 10, 10);

	  //Loop through all ROI and se if its O or X
	  for(int i=0;i<cols*rows;i++){
	    //ROI
		Rec.x=image_width/cols*colcount;
		Rec.y=image_height/rows*rowcount;
		Rec.width=image_width/cols;
		Rec.height=image_height/rows;
		//keep track of ROI
		colcount+=1;
		if (colcount>cols-1){
		   	colcount=0;
		    rowcount+=1;
		    if (rowcount>rows-1){
		    	rowcount=0;
		    }
		}
		//copy to new sub image 
		subImage = org(Rec).clone();
		waitKey(10);

		//debug subimages  
		if(debug_flag){
			namedWindow("show sub", WINDOW_AUTOSIZE);
			imshow("Step show sub", subImage);
			waitKey(100);
		}   
					    
		//filter for red
		inRange(subImage,   cv::Scalar(filter_red_b_low, filter_red_g_low, filter_red_r_low), cv::Scalar(filter_red_b_high, filter_red_g_high, filter_red_r_high), filter_image_red);
		waitKey(10);
		//filter blue
		inRange(subImage,   cv::Scalar(filter_blue_b_low, filter_blue_g_low, filter_blue_r_low), cv::Scalar(filter_blue_b_high, filter_blue_g_high, filter_blue_r_high), filter_image_blue);
		waitKey(10);
		//debug show filter image
		if(debug_flag){
			namedWindow("show filter sub", WINDOW_AUTOSIZE);
			imshow("Step show filterv sub", subImage);
			waitKey(1000);
			namedWindow("show filter sub", WINDOW_AUTOSIZE);
			imshow("Step show filterv sub", filter_image_red);
			waitKey(1000);
		    namedWindow("show filter sub", WINDOW_AUTOSIZE);
			imshow("Step show filterv sub", subImage);
			waitKey(1000);
			namedWindow("show filter sub", WINDOW_AUTOSIZE);
			imshow("Step show filterv sub", filter_image_blue);
			waitKey(1000);
		}
			
		//make bw version
		Mat bw_red=filter_image_red>128;
		Mat bw_blue=filter_image_blue>128;
		//count white pixel
		double white_red= countNonZero(bw_red);
		double white_blue= countNonZero(bw_blue);
		double allpix=filter_image_red.size().width*filter_image_red.size().height;
		//if more than 5% is red and les then 1% blue -> its O-piece
		if(white_red>allpix/95 && white_blue<allpix/99 ){
		    //ROS_INFO_STREAM( " O-piece detected" );
		    gameboard[rowcount][colcount]=2;
		}
		//if more than 5% is blue and les then 1% red -> its X-piece	    
		if(white_red<allpix/99 && white_blue>allpix/95 ){
		    //ROS_INFO_STREAM( " X-piece detected" );	
			gameboard[rowcount][colcount]=1;
		}
		//if less than 1% is red and les then 1% blue -> its no-piece	    
		if(white_red<allpix/99 && white_blue<allpix/99 ){
		    //ROS_INFO_STREAM( " No-piece detected" );	
			gameboard[rowcount][colcount]=0;
		}
		waitKey(10);
			
	  }

	  //Print gameboard
	  for(int i=0;i<rows;i++){
		for(int j=0;j<cols;j++){
			cout << "|" << gameboard[i][j];
		}
		cout << "|" << endl;
		cout << "_______________" << endl;		
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

/*

int main(int argc, char **argv)
{

//TODO: make board detection into service
 //ROS init
	ros::init(argc, argv, "board_detector");
	ros::NodeHandle n;
    ros::Rate loop_rate(5);
    ROS_INFO_THROTTLE(1 , "starting");
    loop_rate.sleep();

 //var init
    //gameboard    
    int rows=7;
    int cols=7;
    //0 = no piece
    //1 = X-piece
    //2 = O-piece
    int** gameboard = new int*[rows];
	for(int i = 0; i < rows; ++i)
    	gameboard[i] = new int[cols];
 
	//  create images for processing
	Mat LoadedImage;
	Mat subImage;
	Mat filter_image_red;
	Mat filter_image_blue;
	//TODO: load from stream
	LoadedImage = imread("/home/tigger77/catkin_ws/src/board_detector/src/mokttt.jpg", IMREAD_COLOR);
 	
 	/*
 	//debug show loaded image
	namedWindow("Step 1 image loaded", WINDOW_AUTOSIZE);
	imshow("Step 1 image loaded", LoadedImage);
	waitKey(100);
    

	/// parameters
	int rowcount=0;
	int colcount=0;
	int image_width=LoadedImage.size().width;
	int image_height=LoadedImage.size().height;
	int filter_red_r_low=50;
	int filter_red_g_low=0;
	int filter_red_b_low=0;
	int filter_red_r_high=255;
	int filter_red_g_high=25;
	int filter_red_b_high=25;
	int filter_blue_r_low=0;
	int filter_blue_g_low=0;
	int filter_blue_b_low=50;
	int filter_blue_r_high=25;
	int filter_blue_g_high=25;
	int filter_blue_b_high=255;
	Rect Rec(0, 0, 10, 10);

	//Loop through all ROI and se if its O or X
	for(int i=0;i<cols*rows;i++){
	    //ROI
	    Rec.x=image_width/cols*colcount;
	    Rec.y=image_height/rows*rowcount;
	    Rec.width=image_width/cols;
	    Rec.height=image_height/rows;
	    //keep track of ROI
	    colcount+=1;
	    if (colcount>cols-1){
	     	colcount=0;
	        rowcount+=1;
	        if (rowcount>rows-1){
		     	rowcount=0;
		    }
	    }
	    //copy to new sub image 
		subImage = LoadedImage(Rec).clone();
		waitKey(10);

		//debug subimages  
		/*   
		namedWindow("show sub", WINDOW_AUTOSIZE);
		imshow("Step show sub", subImage);
		waitKey(100);
	    	
	    
	    //filter for red
	    inRange(subImage,   cv::Scalar(filter_red_b_low, filter_red_g_low, filter_red_r_low), cv::Scalar(filter_red_b_high, filter_red_g_high, filter_red_r_high), filter_image_red);
	    waitKey(10);
	    //filter blue
		inRange(subImage,   cv::Scalar(filter_blue_b_low, filter_blue_g_low, filter_blue_r_low), cv::Scalar(filter_blue_b_high, filter_blue_g_high, filter_blue_r_high), filter_image_blue);
	    waitKey(10);
	    //debug show filter image
	    /*
	    namedWindow("show filter sub", WINDOW_AUTOSIZE);
		imshow("Step show filterv sub", subImage);
		waitKey(1000);
		namedWindow("show filter sub", WINDOW_AUTOSIZE);
		imshow("Step show filterv sub", filter_image_red);
		waitKey(1000);
	    namedWindow("show filter sub", WINDOW_AUTOSIZE);
		imshow("Step show filterv sub", subImage);
		waitKey(1000);
		namedWindow("show filter sub", WINDOW_AUTOSIZE);
		imshow("Step show filterv sub", filter_image_blue);
		waitKey(1000);
		
		
		//make bw version
	    Mat bw_red=filter_image_red>128;
	    Mat bw_blue=filter_image_blue>128;
	    //count white pixel
		double white_red= countNonZero(bw_red);
		double white_blue= countNonZero(bw_blue);
	    double allpix=filter_image_red.size().width*filter_image_red.size().height;
	    //if more than 5% is red and les then 1% blue -> its O-piece
	    if(white_red>allpix/95 && white_blue<allpix/99 ){
	       ROS_INFO_STREAM( " O-piece detected" );
	       gameboard[rowcount][colcount]=2;
		}
		//if more than 5% is blue and les then 1% red -> its X-piece	    
		if(white_red<allpix/99 && white_blue>allpix/95 ){
		   ROS_INFO_STREAM( " X-piece detected" );	
		   gameboard[rowcount][colcount]=1;
		}
		//if less than 1% is red and les then 1% blue -> its no-piece	    
		if(white_red<allpix/99 && white_blue<allpix/99 ){
		   ROS_INFO_STREAM( " No-piece detected" );	
		   gameboard[rowcount][colcount]=0;
		}
		waitKey(10);
		
	}

	//Print gameboard
	for(int i=0;i<rows;i++){
		for(int j=0;j<cols;j++)
		{
			cout << "|" << gameboard[i][j];
		}
		cout << "|" << endl;
		cout << "_______________" << endl;
		
	}
 
 while (ros::ok())
  {
  	ROS_INFO_THROTTLE(1 , "Waiting");
    ros::spinOnce();
    loop_rate.sleep();
  }


}*/


