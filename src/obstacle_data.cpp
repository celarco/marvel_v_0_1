
// %Tag(FULLTEXT)%
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
#include <fstream>

using namespace std;
ofstream logger;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
 

	int frame=1;
	
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
///////////////////////////////////////////8888888888888888


static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
   {
     ros::NodeHandle nh_;
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
       image_pub_ = it_.advertise("/image_converter/output_video", 1);
   
       cv::namedWindow(OPENCV_WINDOW);
     }
   
     ~ImageConverter()
     {
       cv::destroyWindow(OPENCV_WINDOW);
     }
   
  /*   void imageCb(const sensor_msgs::ImageConstPtr& msg)
     {
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
   
       // Draw an example circle on the video stream
       if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
         cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
   
       // Update GUI Window
       cv::imshow(OPENCV_WINDOW, cv_ptr->image);
       cv::waitKey(3);
       
       // Output modified video stream
       image_pub_.publish(cv_ptr->toImageMsg());
     }*/
     
      void imageCb_d(const sensor_msgs::ImageConstPtr& msg)
  {

	double source_po_x=0;
	double source_po_y=0;
	double source_po_z=0;
	double source_po=0;
	
    cv_bridge::CvImagePtr cv_ptr;
	
	//printf ("Decimals: %d %d %d\n", msg->height , msg->width ,msg->step);
	
	double fovh=58.0 / 57.3  ;
	double fovv=45.0 / 57.3 ;
	
	
	//for (uint32_t row=0 ; row <= msg->height ; ++row)
	
	//{
		//for (uint32_t step=0 ; step <= msg->step ; ++step)
		//{
			//source_po=100.0/((msg->data[step*row]+1.0) *( msg->data[step*row] +1.0)* msg->step*msg->height);
			
			//source_po_x=source_po_x-source_po * cos(fovv * (double)(row - msg->height/2)/(double)(msg->height)) * cos ( fovh * (double)(step - msg->step/2)/(double)(msg->step)); 
			//source_po_y=source_po_y+source_po * cos(fovv * (row - msg->height/2)/msg->height) * sin ( fovh * (step - msg->step/2)/msg->step);
			//source_po_z=source_po_z+source_po * sin(fovv * (row - msg->height/2)/msg->height) ;
		//}
	//}
	
	//printf ("X: %f \n", source_po_x);
	//printf ("Y: %f \n", source_po_y);
	//printf ("Z: %f \n", source_po_z);
	//printf ("Decimals: %lu \n", ((msg->data.size())));
	
	//printf ("Decimals: %d \n", msg->data[640*480+50]);
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
	//for (uint32_t step=(msg->width-40) ; step <= msg->width ; ++step)
	//{
		//for (uint32_t step1=(msg->height-40) ; step1 <= msg->height ; ++step1)
		//{maxVal
			//msg->data[step*step1]=0;
		//}
	//}

	

    double maxVal;
    double minVal;
    minMaxLoc(cv_ptr->image, &minVal, &maxVal); //find minimum and maximum intensities
    cv::Mat blur_img;
    cv_ptr->image.convertTo(blur_img, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

	//printf ("maxVal: %f \n", maxVal);
	//printf ("minVal: %f \n", minVal);
	//printf ("100,100 value: %d \n", blur_img.at<unsigned char>(100,100));
	//printf ("100,100 value: %f \n", cv_ptr->image.at<float>(100,100));
	//printf ("column: %d \n",cv_ptr->image.cols);
	//printf ("row: %d \n",cv_ptr->image.rows);
	
	for(int i=0;i<cv_ptr->image.rows;i++)
	{
		for(int j=0;j<cv_ptr->image.cols;j++)
		{
		if (cv_ptr->image.at<float>(i,j)==0){source_po = 1.0/(0.45*0.45);}
		if (cv_ptr->image.at<float>(i,j)!=0)
		{
		source_po = 1.0 /(cv_ptr->image.at<float>(i,j) * cv_ptr->image.at<float>(i,j) * 0.000001);
		}
		source_po_x=source_po_x-source_po * cos(fovv * (double)(i - (cv_ptr->image.rows-1.0)/2)/(double)(cv_ptr->image.rows)) * cos ( fovh * (double)(j - (cv_ptr->image.cols-1.0)/2)/(double)(cv_ptr->image.cols)); 
		source_po_y=source_po_y-source_po * cos(fovv * (double)(i - (cv_ptr->image.rows-1.0)/2)/(double)(cv_ptr->image.rows)) * sin ( fovh * (double)(j - (cv_ptr->image.cols-1.0)/2)/(double)(cv_ptr->image.cols)); 
		source_po_z=source_po_z-source_po * sin(fovv * (double)(i - (cv_ptr->image.rows-1.0)/2)/(double)(cv_ptr->image.rows));
		
		}
	}
	source_po_x = source_po_x / ((double)(cv_ptr->image.rows * cv_ptr->image.cols)) + 1.0;
	source_po_y = source_po_y / ((double)(cv_ptr->image.rows * cv_ptr->image.cols));
	source_po_z = source_po_z / ((double)(cv_ptr->image.rows * cv_ptr->image.cols));
//	printf ("X: %f \n", source_po_x);
//	printf ("Y: %f \n", source_po_y);
//	printf ("Z: %f \n", source_po_z);
	
	    rotate(blur_img,180,blur_img);
	
		 //int fontFace = cv::FONT_HERSHEY_COMPLEX_SMALL;
       double fontScale = 1.5;
       int thickness = 2;
       //cv::Point textOrg(imgW/5, imgH/1.2);
	   std::ostringstream strx;
	   strx<< " X=  "<<source_po_x;
	   std::ostringstream stry;
	   stry<< " Y=  "<<source_po_y;
	   std::ostringstream strz;
	   strz<< " Z=  "<<source_po_z;
	   std::string str=strx.str();
       //std::string someText = source_po_x);
       cv::putText(blur_img, str, cv::Point(10,30),  cv::FONT_HERSHEY_COMPLEX_SMALL, fontScale, cv::Scalar::all(255), thickness,4);
	   str=stry.str();
		cv::putText(blur_img, str, cv::Point(10,60),  cv::FONT_HERSHEY_COMPLEX_SMALL, fontScale, cv::Scalar::all(255), thickness,4);
		str=strz.str();
		cv::putText(blur_img, str, cv::Point(10,90),  cv::FONT_HERSHEY_COMPLEX_SMALL, fontScale, cv::Scalar::all(255), thickness,4);
    
		//std::ostringstream strlog;
	   //strlog<<source_po_x<<" ; "<<source_po_y<<" ; "<<source_po_z<<" \n ";
    logger<<source_po_x<<" ; "<<source_po_y<<" ; "<<source_po_z<<" \n ";
		
		cv::imshow( "OPENCV_WINDOW", blur_img);
	   std::ostringstream framex;
	   framex<< "im"<<frame<<".jpg";
	   std::cout<<  framex.str();
	   cv::imwrite(framex.str(),blur_img);
	   frame=frame+1;
	
	
		
		
    cv::imshow("Blur", blur_img);
    
    
    
    //cv::imshow("Depth", cv_ptr->image);


    cv::waitKey(1);

    //image_pub_.publish(cv_ptr->toImageMsg());
  }
  
  
 };
 
 

   int main(int argc, char** argv)
   {
     ros::init(argc, argv, "image_converter");
     ImageConverter ic;
	
	 logger.open("POTLOG.txt");
	 logger<<" X ; Y ; Z  \n";
	
     ros::spin();
     return 0;
   }
