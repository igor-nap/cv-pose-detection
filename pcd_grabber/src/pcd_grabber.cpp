#include <iostream>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

void
cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud, PointCloudT *cloud_object,
    bool* new_cloud_available_flag)
{
  pcl::copyPointCloud<PointT, PointT>(*callback_cloud, *cloud_object);
  *new_cloud_available_flag = true;
}

int
  main (int argc, char** argv)
{
  //pcl::PointCloud<pcl::PointXYZ> cloud;

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

  /*
  // Add point picking callback to viewer:
  struct callback_args cb_args;
  PointCloudT::Ptr clicked_points_3d (new PointCloudT);
  cb_args.clicked_points_3d = clicked_points_3d;
  cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
  viewer.registerPointPickingCallback (pp_callback, (void*)&cb_args);
  std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;
  */

  // Spin until 'Q' is pressed (to allow ground manual initialization):
  viewer.spin();
  std::cout << "done." << std::endl;

  pcl::io::savePCDFileASCII ("/home/igor/pcds/pcd_grabber_out_1.pcd", *cloud);
  std::cerr << "Saved " << (*cloud).points.size () << " data points to pcd_grabber_out_1.pcd." << std::endl;

  /*
  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
  */

  while (!viewer.wasStopped ())
  {
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }

  return (0);
}

