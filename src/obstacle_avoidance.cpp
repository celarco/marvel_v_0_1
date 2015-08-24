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
#include <marvel_v_0_1/Guidance_Command.h>
#include <marvel_v_0_1/obstacle_avoidance.h>


/**
 * This code is under B.S.D licence
 **/
 

marvel_v_0_1::Guidance_Command guidance_msg;
marvel_v_0_1::obstacle_avoidance vdata;

float vx_uni = 0,vy_uni=0,vz_uni=0;
//
// Guidance message callback function
//

void obstacle_subscribe_callback(const marvel_v_0_1::Guidance_Command::ConstPtr& msg) {

    vx_uni=msg->vx_uni;
    vy_uni=msg->vy_uni;
    vz_uni=msg->vz_uni;
    ros::spinOnce();

}


static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
   {
     ros::NodeHandle nh_;
     ros::NodeHandle n;
     ros::NodeHandle ns;
     ros::Publisher chatter_pub = n.advertise<marvel_v_0_1::obstacle_avoidance>("obstacle_data", 1000);
     ros::Subscriber sub = n.subscribe("guidance_pack", 1000, obstacle_subscribe_callback);
     float value,value1,value2,value3;
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
    source_po_x = source_po_x / ((double)(cv_ptr->image.rows * cv_ptr->image.cols)) + vx_uni;
    source_po_y = source_po_y / ((double)(cv_ptr->image.rows * cv_ptr->image.cols)) + vy_uni;
    source_po_z = source_po_z / ((double)(cv_ptr->image.rows * cv_ptr->image.cols)) + vz_uni;
	printf ("X: %f \n", source_po_x);
	printf ("Y: %f \n", source_po_y);
	printf ("Z: %f \n", source_po_z);
	
    value=source_po_x;
    value1=source_po_y;
    value2=source_po_z;
    vdata.vx=value;
    vdata.vy=value1;
    vdata.vz=value2;
    chatter_pub.publish(vdata);
    ros::spinOnce();
    
    



    cv::imshow("Blur", blur_img);
    //cv::imshow("Depth", cv_ptr->image);


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
       printf( " *               Ali Honari                   * \n" );
       printf( " *        All Right reserved 2015-2016        * \n" );
       printf( " *      Email:Mhkazemi_engineer@yahoo.com     * \n" );
       printf( " *        Email:Celarco.Group@Gmail.com       * \n" );
       printf( " *        Email:Honari.ali@Gmail.com          * \n" );
       printf( " *     AmirKabir University of Technology     * \n" );
       printf( " *   AUT-MAV AUTONOMOUS AIRIAL VEHICLE TEAM   * \n" );
       printf( " *                                            * \n" );
       printf( " *                                            * \n" );
       printf( " *                                            * \n" );
       printf( " *                                            * \n" );
       printf( " ********************************************** \n" );
       printf( " ********************************************** \n" );
     ros::init(argc, argv, "image_converter");
     ros::init(argc, argv, "obstacle_detection");

     ImageConverter ic;
     ros::spin();
     return 0;
   }
