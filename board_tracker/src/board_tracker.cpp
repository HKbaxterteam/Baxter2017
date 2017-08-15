//************************************************
//**********Baxter 2017 Tic-Tac-Toe***************
//*******Nadine Drollinger & Michael Welle********
//************************************************
//*******Camera node - board_tracker*************
//************************************************

//************************************************
// Description: tracks the game board via QR code 
// and publishes the cut out board given the real 
// world qr-code and board measurments in mm.
//************************************************

//Includes
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <zbar.h>  

//Namespace
using namespace cv;  
using namespace std;  
using namespace zbar;  


//Class board_cutout
class board_cutout
{
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_raw_;
  image_transport::Publisher image_pub_cut_;
  
  public:
    //vars for cutting out board
    double rboard_width; //real board width in mm
    double rboard_height; // real board height in mm
    double rqrcode_width; // real QR-code width in mm
    double rqrcode_height; // real QR-code height in mm
    double rqrboard_offset_x; // offset in x direction between qr code and board
    double rqrboard_offset_y; // offset in y direction between qr code and board
    double iqrcode_width; // read out qrcode width from points
    double iqrcode_height;// read out qrcode height from points
    double iqrboard_offset_x; // offset from qrcode to board in image x
    double iqrboard_offset_y; // offset from qrcode to board in image y
    double iboard_width; //image board width
    double iboard_height; //image board heigth
    bool debug_flag;
    //Open cv images
    Mat org, grey, game; 
    Mat M, rotated,rotated2, cropped;
    //board_cutout() : it_(n_),rboard_width(100),rboard_height(100),rqrcode_width(20), rqrcode_height(20), rqrboard_offset_x(-20), rqrboard_offset_y(30), debug_flag(false)
    board_cutout() : it_(n_),rboard_width(420),rboard_height(420),rqrcode_width(90), rqrcode_height(90), rqrboard_offset_x(-400), rqrboard_offset_y(380), debug_flag(false)
    {
      // Subscrive and publisher
      image_sub_raw_ = it_.subscribe("/usb_cam/image_raw", 1, &board_cutout::imageCb, this);
      image_pub_cut_ = it_.advertise("/TTTgame/cut_board", 1);
      
      //debug
      if(debug_flag){
        namedWindow("Input", CV_WINDOW_AUTOSIZE);
        namedWindow("Cut output",CV_WINDOW_AUTOSIZE);
      }
      
    }

    ~board_cutout()
    {
      if(debug_flag){
        destroyWindow("Input");
        destroyWindow("Cut output"); 
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
      //Set up qr code scanner (Zbar)
      ImageScanner scanner;   
      scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);   
      double dWidth = org.cols;  
      double dHeight = org.rows;  
      //cout << "Frame size : " << dWidth << " x " << dHeight << endl;  
      cvtColor(org,grey,CV_BGR2GRAY);  
      int width = org.cols;   
      int height = org.rows;   
      uchar *raw = (uchar *)grey.data;   
      // wrap image data   
      Image image(width, height, "Y800", raw, width * height);   
      // scan the image for barcodes   
      int n = scanner.scan(image);   
      // extract results  
      //TODO: make sure only one barcode is there  
      for(Image::SymbolIterator symbol = image.symbol_begin();   
        symbol != image.symbol_end();   
        ++symbol) {   
          vector<Point> vp;   
          vector<Point> vpgame; 
          //use results
          if(debug_flag)
          {
            cout << "decoded " << symbol->get_type_name() << " symbol " << symbol->get_data() << '"' <<" "<< endl;     
          }  
          int n = symbol->get_location_size();   
          for(int i=0;i<n;i++){   
            vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));   
          }   
          //get new rect for game area

          RotatedRect r = minAreaRect(vp);
          if(vp.size()==4){
            //calculate board position          
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
            vpgame.push_back(vp[0]+Point(iqrboard_offset_x,iqrboard_offset_y));
            vpgame.push_back(vpgame[0]+Point(iboard_width,0));
            vpgame.push_back(vpgame[0]+Point(iboard_width,-iboard_height));
            vpgame.push_back(vpgame[0]+Point(0,-iboard_height));
            RotatedRect rgame = minAreaRect(vpgame);  
            // get angle and size from the bounding box
            float angle = rgame.angle;
            Size rect_size = rgame.size;
            // thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
            if (rgame.angle < -45.) {
                angle += 90.0;
                swap(rect_size.width, rect_size.height);
            }
            // get the rotation matrix 
            M = getRotationMatrix2D(rgame.center, angle, 1);
            // perform the affine transformation
            warpAffine(org, rotated, M, org.size(), INTER_CUBIC);

            //take care of z rotation

            /*
            double phi=0.0;
            double theta=0.00;
            double psi=0.04;

            Mat D = (Mat_<double>(3, 3) <<
              cos(phi), sin(phi),           0,
              -sin(phi), cos(phi), 0,
              0, 0,  1);

            Mat C = (Mat_<double>(3, 3) <<
              1, 0, 0,
              0, cos(theta), sin(theta),
              0, -sin(theta),  cos(theta));

            Mat B = (Mat_<double>(3, 3) <<
              cos(psi), sin(psi),           0,
              -sin(psi), cos(psi), 0,
              0, 0,  1);

              */

/*

            double roll=0.4;
            double pitch=0.00;
            double yaw=0.0;


            Mat Rx = (Mat_<double>(3, 3) <<
              1,          0,           0,
              0, cos(roll), -sin(roll),
              0, sin(roll),  cos(roll));

            Mat Ry = (Mat_<double>(3, 3) <<
              cos(pitch), 0, sin(pitch),
              0, 1,          0,
              -sin(pitch), 0,  cos(pitch));

            Mat Rz = (Mat_<double>(3, 3) <<
              cos(yaw), -sin(yaw), 0,
              sin(yaw),  cos(yaw), 0,
              0,          0,           1);
              

cout << "more matrix " << endl;     
          

            Mat R = Rx*Rz*Ry;
            //Mat R = B*C*D;
            double rotwidth=rotated.cols;
            double rotheight=rotated.rows;
            Mat T = (Mat_<double>(3, 1) << 0,0,1);
            Mat H = (Mat_<double>(3, 1) << rotwidth/2,rotheight/2,1);
            cout << "more T " << endl;  
            T=R*H;
cout << "more H2 " << endl; 
            Mat H2 = (Mat_<double>(3, 3) <<
              1, 0, (rotwidth/2-T.at<double>(0,0)),
              0,  1, (rotheight/2-T.at<double>(1,0)),
              0,          0,           1);
cout << "more W " << endl; 
              Mat W=H2*R;

cout << "more warp " << H2 << endl; 

            Mat HOPE = (Mat_<double>(2, 3) <<
              W.at<double>(0,0), W.at<double>(0,1), 0, 
              W.at<double>(1,0),  W.at<double>(1,1) ,0);
            //Mat trans = A*R*A.inv();
            //warpPerspective(rotated, rotated2, HOPE, rotated.size(),INTER_CUBIC);
            //perspectiveTransform(rotated,rotated2,W);
            warpAffine(rotated,rotated2,HOPE,rotated.size(),INTER_CUBIC);

            
/*
            double roll=0.01;
            double pitch=0.0;
            double yaw=0.4;
            double f=1;
            double cx=0;
            double cy=0;

            // Camera Calibration Intrinsics Matrix
    Mat A2 = (Mat_<double>(3,4) <<
              f, 0, cx, 0,
              0, f, cy, 0,
              0, 0, 1,  0);
    // Inverted Camera Calibration Intrinsics Matrix
    Mat A1 = (Mat_<double>(4,3) <<
              1/f, 0,   -cx/f,
              0,   1/f, -cy/f,
              0,   0,   0,
              0,   0,   1);
    // Rotation matrices around the X, Y, and Z axis
    Mat RX = (Mat_<double>(4, 4) <<
              1, 0,         0,          0,
              0, cos(roll), -sin(roll), 0,
              0, sin(roll), cos(roll),  0,
              0, 0,         0,          1);
    Mat RY = (Mat_<double>(4, 4) <<
              cos(pitch),  0, sin(pitch), 0,
              0,           1, 0,          0,
              -sin(pitch), 0, cos(pitch), 0,
              0,           0, 0,          1);
    Mat RZ = (Mat_<double>(4, 4) <<
              cos(yaw), -sin(yaw), 0, 0,
              sin(yaw),  cos(yaw), 0, 0,
              0,          0,       1, 0,
              0,          0,       0, 1);
    // Translation matrix
    double dx=0;
    double dy=0;
    double dz=1;

    Mat T = (Mat_<double>(4, 4) <<
             1, 0, 0, dx,
             0, 1, 0, dy,
             0, 0, 1, dz,
             0, 0, 0, 1);
    // Compose rotation matrix with (RX, RY, RZ)
    Mat R = RZ * RY * RX;
    // Final transformation matrix
    Mat H = A2 * (T * (R * A1));
    // Apply matrix transformation
    warpPerspective(rotated, rotated2, H, rotated.size(), INTER_LANCZOS4);

*/


    Point2f srcTri[3];
   Point2f dstTri[3];

   /// Set your 3 points to calculate the  Affine Transform
   double gamma=0.1;
   srcTri[0] = Point2f( 541, 458 );
   srcTri[1] = Point2f( 476, 310 );
   srcTri[2] = Point2f( 294, 315);

   dstTri[0] = Point2f( 541, 458);
   dstTri[1] = Point2f( 541,   310-100 );
   dstTri[2] = Point2f( 295-100,  310-100 );

   Mat warp_mat( 2, 3, CV_32FC1 );

   warp_mat = getAffineTransform( srcTri, dstTri );

   warpAffine( rotated, rotated2, warp_mat, rotated.size() );


cout << "more done " << warp_mat << endl; 
            namedWindow("rot", CV_WINDOW_AUTOSIZE);
            namedWindow("org", CV_WINDOW_AUTOSIZE);
            namedWindow("crop", CV_WINDOW_AUTOSIZE);
            imshow("rot", rotated2); //show the frame in "MyVideo" window
            imshow("org", rotated); //show the frame in "MyVideo" window

            // crop the resulting image
            getRectSubPix(rotated2, rect_size, rgame.center, cropped);
            waitKey(10);
            imshow("crop", cropped); //show the frame in "MyVideo" window
            //debug output
            if(debug_flag){
              imshow("Cut output", cropped); //show the frame in "MyVideo" window
              cout << "cut size width : " << cropped.cols << " height " << cropped.rows << endl;  
            }
        }
        //debug
        if(debug_flag){
            //draw lines around detected qr-code
          Point2f pts[4];   
          r.points(pts);   
          for(int i=0;i<4;i++){   
            line(org,pts[i],pts[(i+1)%4],Scalar(255,0,0),3);   
          }   
        }
          
      } 
    //debug
    if(debug_flag){
      imshow("Input", org); //show the show input with lines  
    }    
         
    //publish the image in ros
    cv_bridge::CvImage out_msg;
    out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
    out_msg.header.stamp =ros::Time::now(); // new timestamp
    out_msg.encoding = sensor_msgs::image_encodings::RGB8; // encoding, might need to try some diffrent ones
    out_msg.image    = cropped; 
    image_pub_cut_.publish(out_msg.toImageMsg()); //transfer to Ros image message   

  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "board_tracker");
  board_cutout bc;
  ROS_INFO("board tracker initilized");
  ros::spin();
  return 0;
}
