#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include <marvel_v_0_1/window_detector.h>



/**
 * This tutorial Under B.S.D licence
**/




double some=0;
int imgW = 650;
int imgH = 50;




 using namespace cv;
 using namespace std;



    marvel_v_0_1::window_detector vdata;
  //  ros::init(argc, argv, "position_data");
   // ros::NodeHandle n;


// Return the rotation matrices for each rotation
void rotate(cv::Mat& src, double angle, cv::Mat& dst) {


  cv::Mat r = getRotationMatrix2D(cv::Point2f(), angle, 1.0);

  //4 coordinates of the image
  std::vector<cv::Point2f> corners(4);
  corners[0] = cv::Point2f(0, 0);
  corners[1] = cv::Point2f(0, src.rows);
  corners[2] = cv::Point2f(src.cols, 0);
  corners[3] = cv::Point2f(src.cols, src.rows);

  std::vector<cv::Point2f> cornersTransform(4);
  cv::transform(corners, cornersTransform, r);

  //Copy the 2x3 transformation matrix into a 3x3 transformation matrix
  cv::Mat H = cv::Mat::eye(3, 3, CV_64F);
  for(int i = 0; i < 2; i++) {
    for(int j = 0; j < 3; j++) {
      H.at<double>(i, j) = r.at<double>(i, j);
    }
  }

  double offsetX = 0.0, offsetY = 0.0, maxX = 0.0, maxY = 0.0;
  //Get max offset outside of the image and max width / height
  for(size_t i = 0; i < 4; i++) {
    if(cornersTransform[i].x < offsetX) {
      offsetX = cornersTransform[i].x;
    }

    if(cornersTransform[i].y < offsetY) {
      offsetY = cornersTransform[i].y;
    }

    if(cornersTransform[i].x > maxX) {
      maxX = cornersTransform[i].x;
    }

    if(cornersTransform[i].y > maxY) {
      maxY = cornersTransform[i].y;
    }
  }

  offsetX = -offsetX;
  offsetY = -offsetY;
  maxX += offsetX;
  maxY += offsetY;

  cv::Size size_warp(maxX, maxY);

  //Create the transformation matrix to be able to have all the pixels
  cv::Mat H2 = cv::Mat::eye(3, 3, CV_64F);
  H2.at<double>(0,2) = offsetX;
  H2.at<double>(1,2) = offsetY;

  warpPerspective(src, dst, H2*H, size_warp);
}







static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
   {



     ros::NodeHandle nh_;
     ros::NodeHandle n;
     ros::Publisher chatter_pub = n.advertise<marvel_v_0_1::window_detector>("window_postion", 1000);
     float value,value1,value2,value3,value4,value5;


     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_;
     image_transport::Publisher image_pub_;
     
   public:
     ImageConverter()
       : it_(nh_)
     {
       // Subscrive to input video feed and publish output video feed
       image_sub_ = it_.subscribe("/camera/depth/image_raw", 1, 
    
         &ImageConverter::imageCb_d, this);
      // image_pub_ = it_.advertise("/image_converter/output_video", 1);
   
       //cv::namedWindow(OPENCV_WINDOW);
     }
   
     ~ImageConverter()
     {
       cv::destroyWindow(OPENCV_WINDOW);
     }
   
  
     
      void imageCb_d(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;


	
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	int threshold_value = 0;
	int threshold_type = 3;
	int const max_value = 255;
	int const max_type = 4;
	static int max_BINARY_value = 255;
	static int binary_tresh=58;
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	int morph_elem = 2;
	int morph_size = 22;
	int morph_operator = 1;
   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	static int thresh = 100;
	int max_thresh = 255;
	RNG rng(12345);
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	int ilation_elem =2 ,max_elem=21 , dilation_elem=2 ,dilation_size=21;
	int const max_kernel_size = 21;
	
	int erosion_elem=2,erosion_size=21;
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    double maxVal=7000.0;
    double minVal=450.0 ,Minav  ;
    
    Scalar debug ,binary2_q;
    //======================= normalize coefficient ========================
    double alpha,beta;
    int dtype ,NORM_L2,mask;
    Mat Asghar;
  
    
    createTrackbar( " Canny thresh:", "binary", &thresh, max_thresh, 0 );
   
   // minMaxLoc(cv_ptr->image, &minVal, &maxVal); //find minimum and maximum intensities
    cv::Mat blur_img ,binary;
    cv_ptr->image.convertTo(blur_img, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
         Mat binary2 ;
        
     minMaxLoc(blur_img, &minVal, &maxVal); //find minimum and maximum intensities
     debug=cv::mean(blur_img);
     // meanStdDev(blur_img, Minav, minAV , CvArr** mask=NULL);
     
     rotate(blur_img, 180, blur_img);

      //equalizeHist( blur_img, binary );
      //imshow("histogram",binary);
      
      blur(blur_img , binary , Size(3 , 3) , Point(-1,-1), 0 );
      
  //    imshow ("blur shode",binary);
      
      /*
     cout << "maxval:   " << maxVal << endl ;
     cout << "minval:   " << minVal << endl;
     Minav=(maxVal+minVal)/2.0;
     
     cout << "miangin:     " << Minav << endl  ;
     
     
     
     cout << "debug:   " << debug << endl;
     cout << "binary2_q:      " << binary2_q << endl;
     
     cout << "================================" <<endl;

*/







//  imshow("aks asli",blur_img);

  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( blur_img, binary, element );

//imshow("dialation",binary);
 

 
if (maxVal-debug[0] > binary_tresh)	
   cv::threshold( binary, binary,(debug[0]+15) , 255,0);
   else 
      cv::threshold( binary, binary, 0, 255,0);

binary2=binary;   
 //  imshow("binary",binary);
   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   
  // imshow ("origin binary",binary);
  
   
  /* 
  // Since MORPH_X : 2,3,4,5 and 6
  int operation = morph_operator + 2;

  Mat pashm = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );

  /// Apply the specified morphology operation
  morphologyEx( blur_img, binary, operation, pashm );
  
  imshow ("Morphology allgorithm" ,binary);
  
   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   
   
    cv::threshold( binary, binary, binary_tresh, 255,0);
    imshow ("2 binary",binary);
   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
 */
 
 /*
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( binary, binary, element );
 */
 /*
 
 //erosion allgorithm for ...!

   int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  
  /// Apply the erosion operation
  erode( binary, binary, element );
   
  */
   
    Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
  Canny( binary, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

double x[10000];
double y[10000];

 /// Get the moments
  vector<Moments> mu(contours.size() );




  for( int i = 0; i < contours.size(); i++ )
     { mu[i] = moments( contours[i], false );


  }


  ///  Get the mass centers:
  vector<Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
     
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

 createTrackbar( " binary_tresh:", "binary", &binary_tresh, max_thresh, 0 );
 


if (maxVal-debug[0] > binary_tresh){
	
  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );



  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     }
     



    
    /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
  //printf("\t Info: Area and Contour Length \n");
  for( int i = 0; i< contours.size(); i++ )
     {
     //  printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
       circle( drawing, mc[i], 4, color, -1, 8, 0 );

     }

 imshow( "Origin Contour", drawing );



}

double counter;





if (maxVal-debug[0] > binary_tresh){


 for( int i = 0; i < contours.size(); i++ )
 {

     counter=counter+1;

      if (counter==20){
          some=0;
          counter=0;
      }


   if ( (arcLength( contours[i], true ) >= some) && (contourArea(contours[i])>= some) )
     {

       /// Draw contours
       Mat drawing1 = Mat::zeros( canny_output.size(), CV_8UC3 );

       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing1, contours, i, color, 2, 8, hierarchy, 0, Point() );

       circle( drawing1, mc[i], 4, color, -1, 8, 0 );




       int fontFace = FONT_HERSHEY_COMPLEX_SMALL;
       double fontScale = 1.5;
       int thickness = 2;
       Point textOrg(imgW/5, imgH/1.2);
       string someText = "AUTMAV target locked!";
       putText(drawing1, someText, textOrg, fontFace, fontScale, Scalar::all(255), thickness,8);


          imshow( "Target Lock", drawing1);



   some=arcLength( contours[i] , true );
   x[i]=mu[i].m10/mu[i].m00;
   y[i]=mu[i].m01/mu[i].m00;
   double orientation = 0.5*atan(2*mu[i].m11 / (mu[i].m20 - mu[i].m02)); //get say angle
/*
   cout << "x        :" << x[i] << "========" << some <<endl;
   cout << "y        :" << y[i] << "========"<< some <<endl;
   cout << "Delta X  :" << x[i]-(640/2) << endl;
   cout << "Delta Y  :" << y[i]-(480/2) << endl;

   cout << "Angle (Pitch)           :" << orientation*57.3 <<  endl;
*/

   value=x[i];
   value1=y[i];
   value2=(x[i]-(640/2));
   value3=(y[i]-(480/2));
   value4=(orientation*57.3);
   vdata.x=value;
   vdata.y=value1;
   vdata.deltax=value2;
   vdata.deltay=value3;
   vdata.pith=value4;

   chatter_pub.publish(vdata);
   ros::spinOnce();


   /*

  */

    }
//else cout << "not found" << endl;


 }                                              //for end




}                                               //end if

    //cv::imshow("ax asli", blur_img);
    //cv::imshow("binary", binary);
   

     
//    imshow( "Dilation Demo", dilation_dst );

    cv::waitKey(1);

    //image_pub_.publish(cv_ptr->toImageMsg());
  }
  
  
 };
 
 

   int main(int argc, char** argv)
   {
   printf( " ********************************************** \n" );
   printf( " ********************************************** \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *     This program debug and Develop by      * \n" );
   printf( " *          Mohammad Hossein Kazemi           * \n" );
   printf( " *               Ali Jameie                   * \n" );
   printf( " *        All Right reserved 2015-2016        * \n" );
   printf( " *      Email:Mhkazemi_engineer@yahoo.com     * \n" );
   printf( " *        Email:Celarco.Group@Gmail.com       * \n" );
   printf( " *     AmirKabir University of Technology     * \n" );
   printf( " *   AUT-MAV AUTONOMOUS AIRIAL VEHICLE TEAM   * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " ********************************************** \n" );
   printf( " ********************************************** \n" );
   
       ros::init(argc, argv, "image_converter");
       ros::init(argc, argv, "position_data");



     ImageConverter ic;
     ros::spin();
     return 0;
   }
