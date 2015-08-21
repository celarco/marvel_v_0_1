    #include <ros/ros.h>
    #include <image_transport/image_transport.h>
    #include <cv_bridge/cv_bridge.h>
    #include <sensor_msgs/image_encodings.h>
    #include <opencv2/imgproc/imgproc.hpp>
    #include <opencv2/highgui/highgui.hpp>
    #include "opencv2/opencv.hpp"
    #include <iostream>



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
     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_;
     image_transport::Publisher image_pub_;
     
   public:
     ImageConverter()
       : it_(nh_)
     {
       // Subscrive to input video feed and publish output video feed
       image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, 
         &ImageConverter::imageCb, this);
      // image_pub_ = it_.advertise("/image_converter/output_video", 1);
  
       cv::namedWindow(OPENCV_WINDOW);
     }
   
     ~ImageConverter()
     {
       cv::destroyWindow(OPENCV_WINDOW);
     }
   
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
       
   //imshow("pashmak",cv_ptr->image);
     cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(), 1, 1);
       //imshow("input", cv_ptr->image);
       
       cv::Mat rotated,rotated1,rotated2;
      
  
    rotate(cv_ptr->image, 180, rotated);
    rotate(cv_ptr->image, 90, rotated1);
    rotate(cv_ptr->image, 270, rotated2);
 imshow("rotate",rotated);
  
   
       // Update GUI Window
       //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
       cv::waitKey(3);
       
       // Output modified video stream
       image_pub_.publish(cv_ptr->toImageMsg());
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
   printf( " *               ALI Jameei                   * \n" );
   printf( " *        All Right reserved 2015-2016        * \n" );
   printf( " *      Email:Mhkazemi_Engineer@Yahoo.com     * \n" );
   printf( " *           Celarco.Group@Gmail.com          * \n" );
   printf( " *     AmirKabir University of Technology     * \n" );
   printf( " *    AUT-MAV AUTONOMOUS AIRIAL VEHICLE TEAM  * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " ********************************************** \n" );
   printf( " ********************************************** \n" );
    ros::init(argc, argv, "image_converter");
     ImageConverter ic;

     ros::spin();
     return 0;
   }
