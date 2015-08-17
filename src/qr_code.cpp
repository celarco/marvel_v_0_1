 #include <opencv2/highgui/highgui.hpp>  
 #include <opencv2/imgproc/imgproc.hpp>  
 #include "zbar.h" 
 #include <iostream> 
 #include <marvel_v_0_1/QRms.h>
 #include "ros/ros.h"
 #include "std_msgs/String.h"
 #include <string>
 
 
  
 using namespace cv;  
 using namespace std;  
 using namespace zbar; 
 
 
 
  
 //g++ main.cpp /usr/local/include/ /usr/local/lib/ -lopencv_highgui.2.4.8 -lopencv_core.2.4.8  
 
 
 
 marvel_v_0_1::QRms vdata;
 
 
 
 
 int main(int argc, char* argv[])  
 {  
	 
   printf( " ********************************************** \n" );
   printf( " ********************************************** \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *       This program is Developed by         * \n" );
   printf( " *          Mohammad Hossein Kazemi           * \n" );
   printf( " *        All Right reserved 2015-2016        * \n" );
   printf( " *      Email:Mhkazemi_engineer@yahoo.com     * \n" );
   printf( " *     AmirKabir University of Technology     * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " *                                            * \n" );
   printf( " ********************************************** \n" );
   printf( " ********************************************** \n" );
   

   VideoCapture cap(1); // open the video camera  

   
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

      ros::init(argc, argv, "QRcode");
      ros::NodeHandle n;
      ros::Publisher chatter_pub = n.advertise<marvel_v_0_1::QRms>("MHData", 1000);
      std::string Debug;
    
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
     cvtColor(frame,grey,CV_BGR2GRAY);  
     int width = frame.cols;   
     int height = frame.rows;   
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
     // do something useful with results   
     
     cout << "decoded " << symbol->get_type_name() << " symbol \"" << symbol->get_data() << '"' <<" "<< endl;   
     
     
     
     
     
      Debug=symbol->get_data();
      std::stringstream stream(Debug);
      std::string Debug2;
      
      float value1,value2;
      stream >> value1;
      stream >> Debug2;
      stream >> value2 ;
      vdata.x=value1;
      vdata.y=value2;
   
     
    //   imshow("MyVideo", frame); //show the frame in "MyVideo" window  
    
       int n = symbol->get_location_size();   
       for(int i=0;i<n;i++){   
         vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));   
       }   
       RotatedRect r = minAreaRect(vp);   
       Point2f pts[4];   
       r.points(pts);   
       for(int i=0;i<4;i++){   
         line(frame,pts[i],pts[(i+1)%4],Scalar(255,0,0),3);   
       }   
      
     }
       
     //imshow("MyVideo", frame); //show the frame in "MyVideo" window  
     
     chatter_pub.publish(vdata);
     ros::spinOnce();
     
     
   }  
   return 0;  
 }  

