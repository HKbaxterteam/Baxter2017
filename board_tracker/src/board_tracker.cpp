//************************************************
//**********Baxter 2017 Tic-Tac-Toe***************
//*******Nadine Drollinger & Michael Welle********
//************************************************
//*******Camera node - board tracker*************
//************************************************

//************************************************
//Description: tracks the game board via QR code and publishes the cut out board
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
//using namespace cv;
//using namespace std;


//plan:

//subscribe to QR-code position provided by visp_auto_tracker
//take input image and 


//Class

#include <opencv2/highgui/highgui.hpp>  
 #include <opencv2/imgproc/imgproc.hpp>  
 #include <zbar.h>  
 #include <iostream>  
 using namespace cv;  
 using namespace std;  
 using namespace zbar;  
 //g++ main.cpp /usr/local/include/ /usr/local/lib/ -lopencv_highgui.2.4.8 -lopencv_core.2.4.8  
 int main(int argc, char* argv[])  
 {  
   VideoCapture cap(0); // open the video camera no. 0  
   // cap.set(CV_CAP_PROP_FRAME_WIDTH,800);  
   // cap.set(CV_CAP_PROP_FRAME_HEIGHT,640);  
   if (!cap.isOpened()) // if not success, exit program  
   {  
     cout << "Cannot open the video cam" << endl;  
     return -1;  
   }  
   ImageScanner scanner;   
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);   
   double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video  
   double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video  
   cout << "Frame size : " << dWidth << " x " << dHeight << endl;  
   namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"  
   while (1)  
   {  
     Mat frame;  
     bool bSuccess = cap.read(frame); // read a new frame from video  
      if (!bSuccess) //if not success, break loop  
     {  
        cout << "Cannot read a frame from video stream" << endl;  
        break;  
     }  
     Mat grey;  
     Mat game;
     Mat org=frame;
     cvtColor(frame,grey,CV_BGR2GRAY);  
     int width = frame.cols;   
     int height = frame.rows;   
     //vars for cutting out board
        double rboard_width; //real board width in mm
        double rboard_height; // real board height in mm
        double rqrcode_width; // real QR-code width in mm
        double rqrcode_height; // real QR-code height in mm
        double rqrboard_offset_x; // offset in x direction between qr code and board
        double rqrboard_offset_y; // offset in y direction between qr code and board
        double iqrcode_width; // read out qrcode width from points
        double iqrcode_height;// read out qrcode height from points
        double iqrboard_offset_x;
        double iqrboard_offset_y;
        double iboard_width;
        double iboard_height;

     uchar *raw = (uchar *)grey.data;   
     // wrap image data   
     Image image(width, height, "Y800", raw, width * height);   
     // scan the image for barcodes   
     int n = scanner.scan(image);   
     // extract results   
     for(Image::SymbolIterator symbol = image.symbol_begin();   
     symbol != image.symbol_end();   
     ++symbol) {   
         vector<Point> vp;   
     	 vector<Point> vpgame; 
     // do something useful with results   
     cout << "decoded " << symbol->get_type_name() << " symbol " << symbol->get_data() << '"' <<" "<< endl;   
       int n = symbol->get_location_size();   
       for(int i=0;i<n;i++){   
         vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));   
       }   
       cout << "found Qr at: p0  " << vp[0] << " p1 " << vp[1]  << " p2 " << vp[2] << " p3 " << vp[3] <<'"' <<" "<< endl;   
       cout << "vp size  " << vp.size() << endl;   
     
       RotatedRect r = minAreaRect(vp);   
       //get new rect for game area
       if(vp.size()==4){

        //calculate board position
        rboard_width = 100; //real board width in mm
        rboard_height = 100; // real board height in mm
        rqrcode_width = 20; // real QR-code width in mm
        rqrcode_height = 20; // real QR-code height in mm
        rqrboard_offset_x = -20; // offset in x direction between qr code and board
        rqrboard_offset_y = 30; // offset in y direction between qr code and board

        //read out image qr-code width and height
        iqrcode_width = vp[3].x- vp[0].x; // read out qrcode width from points
        iqrcode_height= vp[1].y-vp[0].y; // read out qrcode height from points

        
        //calc image qrcode to board offsets
        iqrboard_offset_x = (iqrcode_width*rqrboard_offset_x)/rqrcode_width;
        iqrboard_offset_y = (iqrcode_height*rqrboard_offset_y)/rqrcode_height;

        // calc image board width and héight
        iboard_width=(rboard_width*iqrcode_height)/rqrcode_width;
        iboard_height=(rboard_height*iqrcode_height)/rqrcode_height;

        //set 4 points to cut out the image

       vpgame.push_back(vp[0]+Point(iqrboard_offset_x,iqrboard_offset_x));
       vpgame.push_back(vpgame[0]+Point(iboard_width,0));
       vpgame.push_back(vpgame[0]+Point(iboard_width,-iboard_height));
       vpgame.push_back(vpgame[0]+Point(0,-iboard_height));
       


      
       RotatedRect rgame = minAreaRect(vpgame);  
        // matrices we'll use
        Mat M, rotated, cropped;
        // get angle and size from the bounding box
        float angle = rgame.angle;
        Size rect_size = rgame.size;
        // thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
        if (rgame.angle < -45.) {
            angle += 90.0;
            swap(rect_size.width, rect_size.height);
        }
        // get the rotation matrix
        M = getRotationMatrix2D(rgame.center, angle, 1.0);
        // perform the affine transformation
        warpAffine(org, rotated, M, org.size(), INTER_CUBIC);
        // crop the resulting image
        getRectSubPix(rotated, rect_size, rgame.center, cropped);
       waitKey(10);
		imshow("Mygame", cropped); //show the frame in "MyVideo" window
       
       }
       
       Point2f pts[4];   
       r.points(pts);   
       for(int i=0;i<4;i++){   
         line(frame,pts[i],pts[(i+1)%4],Scalar(255,0,0),3);   
       }   
       //cout<<"Angle: "<<r.angle<<endl;   
     }   
     imshow("MyVideo", frame); //show the frame in "MyVideo" window  
     if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop  
     {  
       cout << "esc key is pressed by user" << endl;  
       break;   
     }  
   }  
   return 0;  
 }  


/*
int main(int argc, char **argv)
{

//TODO: make board detection into service
 //ROS init
	ros::init(argc, argv, "board_tracker");
	ros::NodeHandle n;
    ros::Rate loop_rate(5);
    ROS_INFO_THROTTLE(1 , "starting");
    loop_rate.sleep();
    




}*/