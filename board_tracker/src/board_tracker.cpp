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

    static double rad2Deg(double rad){return rad*(180/M_PI);}//Convert radians to degrees
static double deg2Rad(double deg){return deg*(M_PI/180);}//Convert degrees to radians

void warpMatrix(Size   sz,
                double theta,
                double phi,
                double gamma,
                double scale,
                double fovy,
                Mat&   M,
                vector<Point2f>* corners){
    double st=sin(deg2Rad(theta));
    double ct=cos(deg2Rad(theta));
    double sp=sin(deg2Rad(phi));
    double cp=cos(deg2Rad(phi));
    double sg=sin(deg2Rad(gamma));
    double cg=cos(deg2Rad(gamma));

    double halfFovy=fovy*0.5;
    double d=hypot(sz.width,sz.height);
    double sideLength=scale*d/cos(deg2Rad(halfFovy));
    double h=d/(2.0*sin(deg2Rad(halfFovy)));
    double n=h-(d/2.0);
    double f=h+(d/2.0);

    Mat F=Mat(4,4,CV_64FC1);//Allocate 4x4 transformation matrix F
    Mat Rtheta=Mat::eye(4,4,CV_64FC1);//Allocate 4x4 rotation matrix around Z-axis by theta degrees
    Mat Rphi=Mat::eye(4,4,CV_64FC1);//Allocate 4x4 rotation matrix around X-axis by phi degrees
    Mat Rgamma=Mat::eye(4,4,CV_64FC1);//Allocate 4x4 rotation matrix around Y-axis by gamma degrees

    Mat T=Mat::eye(4,4,CV_64FC1);//Allocate 4x4 translation matrix along Z-axis by -h units
    Mat P=Mat::zeros(4,4,CV_64FC1);//Allocate 4x4 projection matrix

    //Rtheta
    Rtheta.at<double>(0,0)=Rtheta.at<double>(1,1)=ct;
    Rtheta.at<double>(0,1)=-st;Rtheta.at<double>(1,0)=st;
    //Rphi
    Rphi.at<double>(1,1)=Rphi.at<double>(2,2)=cp;
    Rphi.at<double>(1,2)=-sp;Rphi.at<double>(2,1)=sp;
    //Rgamma
    Rgamma.at<double>(0,0)=Rgamma.at<double>(2,2)=cg;
    Rgamma.at<double>(0,2)=sg;Rgamma.at<double>(2,0)=sg;

    //T
    T.at<double>(2,3)=-h;
    //P
    P.at<double>(0,0)=P.at<double>(1,1)=1.0/tan(deg2Rad(halfFovy));
    P.at<double>(2,2)=-(f+n)/(f-n);
    P.at<double>(2,3)=-(2.0*f*n)/(f-n);
    P.at<double>(3,2)=-1.0;
    //Compose transformations
    F=P*T*Rphi*Rtheta*Rgamma;//Matrix-multiply to produce master matrix

    //Transform 4x4 points
    double ptsIn [4*3];
    double ptsOut[4*3];
    double halfW=sz.width/2, halfH=sz.height/2;

    ptsIn[0]=-halfW;ptsIn[ 1]= halfH;
    ptsIn[3]= halfW;ptsIn[ 4]= halfH;
    ptsIn[6]= halfW;ptsIn[ 7]=-halfH;
    ptsIn[9]=-halfW;ptsIn[10]=-halfH;
    ptsIn[2]=ptsIn[5]=ptsIn[8]=ptsIn[11]=0;//Set Z component to zero for all 4 components

    Mat ptsInMat(1,4,CV_64FC3,ptsIn);
    Mat ptsOutMat(1,4,CV_64FC3,ptsOut);

    perspectiveTransform(ptsInMat,ptsOutMat,F);//Transform points

    //Get 3x3 transform and warp image
    Point2f ptsInPt2f[4];
    Point2f ptsOutPt2f[4];

    for(int i=0;i<4;i++){
        Point2f ptIn (ptsIn [i*3+0], ptsIn [i*3+1]);
        Point2f ptOut(ptsOut[i*3+0], ptsOut[i*3+1]);
        ptsInPt2f[i]  = ptIn+Point2f(halfW,halfH);
        ptsOutPt2f[i] = (ptOut+Point2f(1,1))*(sideLength*0.5);
    }

    M=getPerspectiveTransform(ptsInPt2f,ptsOutPt2f);

    //Load corners vector
    if(corners){
        corners->clear();
        corners->push_back(ptsOutPt2f[0]);//Push Top Left corner
        corners->push_back(ptsOutPt2f[1]);//Push Top Right corner
        corners->push_back(ptsOutPt2f[2]);//Push Bottom Right corner
        corners->push_back(ptsOutPt2f[3]);//Push Bottom Left corner
    }
}

void warpImage(const Mat &src,
               double    theta,
               double    phi,
               double    gamma,
               double    scale,
               double    fovy,
               Mat&      dst,
               Mat&      M,
               vector<Point2f> &corners){
    double halfFovy=fovy*0.5;
    double d=hypot(src.cols,src.rows);
    double sideLength=scale*d/cos(deg2Rad(halfFovy));

    warpMatrix(src.size(),theta,phi,gamma, scale,fovy,M,&corners);//Compute warp matrix
    warpPerspective(src,dst,M,Size(sideLength,sideLength));//Do actual image warp
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
            //cv::Point2f center(org.cols/2.0, org.rows/2.0);
    
            //M.at<double>(0,2) += org.cols*1.5 ;
            //M.at<double>(1,2) += org.rows*1.5;

            // perform the affine transformation
            warpAffine(org, rotated, M, org.size(), INTER_CUBIC);

Mat m, disp, warp;
Rect myROI(0, 130, 550, 300);
Mat croppedRef(rotated, myROI);
croppedRef.copyTo(disp);


    vector<Point2f> corners;
    cout << "starting warping" << endl; 
        warpImage(disp, 0, -70, 0, 1, 78, rotated2, warp, corners);
        cout << "done warping" << endl;
        //namedWindow("disp", WINDOW_NORMAL);
        //resizeWindow("disp", 800,800);
        //imshow("disp", disp);
    

cout << "show stuff" << endl;
//rotated2=rotated;
            namedWindow("rot", WINDOW_NORMAL);
            namedWindow("org", WINDOW_NORMAL);
            namedWindow("crop", WINDOW_NORMAL);
            namedWindow("disp", WINDOW_NORMAL);
            resizeWindow("rot", 800,800);
            resizeWindow("org", 800,800);
            imshow("disp", disp); //show the frame in "MyVideo" window
            
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
