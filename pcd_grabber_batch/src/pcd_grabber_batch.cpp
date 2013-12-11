#include <stdio.h>

#include <iostream>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>

// OpenCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/// Namespaces

namespace enc = sensor_msgs::image_encodings;

using namespace cv;

// PCL

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const int BOUNDING_WIDTH = 240;
const int BOUNDING_HEIGHT = 240;

const int KINECT_X = 640;
const int KINECT_Y = 480;

const int BOUNDING_X = KINECT_X/2 - BOUNDING_WIDTH/2;
const int BOUNDING_Y = KINECT_Y/2 - BOUNDING_HEIGHT/2;

static const std::string OPENCV_WINDOW = "Image window";

//rgb_image_sub_ = it_.subscribe("/camera/depth_registered/image", 1, &VisualEstimator::msgToImageRGB, this);


// PCL to OpenCV

void PC_to_Mat(PointCloudT::Ptr &cloud, cv::Mat &result){

  if (cloud->isOrganized()) {
    std::cout << "PointCloud is organized..." << std::endl;

    result = cv::Mat(cloud->height, cloud->width, CV_8UC3);

    if (!cloud->empty()) {

      for (int h=0; h<result.rows; h++) {
        for (int w=0; w<result.cols; w++) {
            PointT point = cloud->at(w, h);

            Eigen::Vector3i rgb = point.getRGBVector3i();

            result.at<cv::Vec3b>(h,w)[0] = rgb[2];
            result.at<cv::Vec3b>(h,w)[1] = rgb[1];
            result.at<cv::Vec3b>(h,w)[2] = rgb[0];
        }
      }
    }
  }
}



// usage: drawBoundingBox(cv_ptr->image, BOUNDING_X, BOUNDING_Y, BOUNDING_WIDTH, BOUNDING_HEIGHT)
void drawBoundingBox(cv::Mat &img, int x, int y, int width, int height){
  cv::Point P1 = cv::Point(x,y);
  cv::Point P2 = cv::Point(x+width,y+height);
  rectangle(img,P1,P2, cv::Scalar(0,255,0), 2, 8, 0); //(Mat& img, Rect rec, const Scalar& color, int thickness=1, int lineType=8, int shift=0 )
}

//cv_bridge::CvImagePtr cv_ptr;
int user_input = 0;

void imageCb_rgb(const sensor_msgs::ImageConstPtr& msg){
  //static cv_bridge::CvImagePtr cv_ptr;
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg);//, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Draw a bounding box on the video stream
  //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
  drawBoundingBox(cv_ptr->image, BOUNDING_X, BOUNDING_Y, BOUNDING_WIDTH, BOUNDING_HEIGHT);

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  user_input = cv::waitKey(3);

  // Output modified video stream
  //image_pub.publish(cv_ptr->toImageMsg());
}

PointCloudT::Ptr cloud_ptr_global;

void imageCb_rgbd(const sensor_msgs::PointCloud2ConstPtr& rgbd_msg){
  pcl::fromROSMsg (*rgbd_msg, *cloud_ptr_global);
}

// PCL viewer //
//pcl::visualization::PCLVisualizer viewer("PCL Viewer");

void cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud, PointCloudT *cloud_object,
    bool* new_cloud_available_flag) {

  pcl::copyPointCloud<PointT, PointT>(*callback_cloud, *cloud_object);
  *new_cloud_available_flag = true;
}

int main (int argc, char** argv){
  //pcl::PointCloud<pcl::PointXYZ> cloud;

  // OpenCV
  cv::namedWindow(OPENCV_WINDOW);

  // ROS
  ros::init(argc, argv, "pcd_batch_grabber");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub_rgb = it.subscribe("/camera/rgb/image_color", 1, imageCb_rgb);
  ros::Subscriber image_sub_rgbd = nh.subscribe("/camera/depth_registered/points", 1, imageCb_rgbd);
  //image_pub = nh.advertise("/pcd_batch_grabber/output_video", 1);

  
  PointCloudT cloud;
  PointCloudT::Ptr cloud_ptr (&cloud);
  cloud_ptr_global = cloud_ptr;
  

  #if 0
  PointCloudT cloud_obj;
  PointCloudT::Ptr cloud (new PointCloudT);
  bool new_cloud_available_flag = false;
  pcl::Grabber* interface = new pcl::OpenNIGrabber();
  boost::function<void (const PointCloudT::ConstPtr&)> f =
        boost::bind (&cloud_cb_, _1, &cloud_obj, &new_cloud_available_flag);
  interface->registerCallback (f);
  interface->start ();

  while(!new_cloud_available_flag) 
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
  pcl::copyPointCloud<PointT, PointT>(cloud_obj, *cloud);
  new_cloud_available_flag = false;
  #endif

  /*
  // Read Kinect live stream:
  PointCloudT cloud_obj;
  PointCloudT::Ptr cloud (new PointCloudT);
  bool new_cloud_available_flag = false;
  pcl::Grabber* interface = new pcl::OpenNIGrabber();
  boost::function<void (const PointCloudT::ConstPtr&)> f =
        boost::bind (&cloud_cb_, _1, &cloud_obj, &new_cloud_available_flag);
  interface->registerCallback (f);
  interface->start ();

  // Wait for the first frame:
  while(!new_cloud_available_flag) 
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
  pcl::copyPointCloud<PointT, PointT>(cloud_obj, *cloud);
  new_cloud_available_flag = false;

  // Display pointcloud:
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);
  */
  

  /*
  // Add point picking callback to viewer:
  struct callback_args cb_args;
  PointCloudT::Ptr clicked_points_3d (new PointCloudT);
  cb_args.clicked_points_3d = clicked_points_3d;
  cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
  viewer.registerPointPickingCallback (pp_callback, (void*)&cb_args);
  std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;
  */

  /*
  // Spin until 'Q' is pressed (to allow ground manual initialization):
  viewer.spin();
  std::cout << "done." << std::endl;

  pcl::io::savePCDFileASCII ("/home/igor/pcds/pcd_grabber_out_1.pcd", *cloud);
  std::cerr << "Saved " << (*cloud).points.size () << " data points to pcd_grabber_out_1.pcd." << std::endl;
  */

  /*
  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
  */

  int count = 0;
  string pcd_file_base ="/home/igor/pcds/pcd_grabber_batch/pcd_grabber_out_";
  string pcd_file_end = ".pcd";
  string zero_str = "0";
  string leading_zeros = "";

  string pcd_file_out;
  int KEY_QUIT = 1048689;
  bool flag_quit = false;

  while (ros::ok() && !flag_quit){
    //drawBoundingBox(cv_ptr->image, BOUNDING_X, BOUNDING_Y, BOUNDING_WIDTH, BOUNDING_HEIGHT);

  // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);

    //user_input = cvWaitKey(30);;   // call your non-blocking input function
    
    if (user_input == KEY_QUIT){
      flag_quit = true;
    } else if (user_input != KEY_QUIT && user_input != -1 && user_input != 0){
      // if (new_cloud_available_flag)
      if (count >= 100) {
        leading_zeros = "";}
      else if (count >= 10) {
        leading_zeros = "0";}
      else {
        leading_zeros = "00";
      }

      pcd_file_out = pcd_file_base + leading_zeros + boost::to_string(count) + pcd_file_end;
      //pcl::copyPointCloud<PointT, PointT>(cloud_obj, *cloud);
      pcl::io::savePCDFileASCII (pcd_file_out, *cloud_ptr_global);
      std::cout <<  user_input << std::endl;
      std::cerr << "Saved " << (*cloud_ptr_global).points.size () << " data points to PCD file #" << leading_zeros << count << std::endl;
      user_input = -1;
      count++;
    } /*else {
      // do nothing flag_quit = true;
    }*/
    

    

    ros::spinOnce();
  }

  /*
  while (!viewer.wasStopped ())
  {
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }
  */

  return (0);
}

