#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
//#include <pcl/people/ground_based_people_detection_app.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// ICP

#include <pcl/registration/icp.h>

// Boost
#include <boost/filesystem.hpp>

// ROS+OpenCV libs

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// OpenCV Definitions

namespace enc = sensor_msgs::image_encodings;

using namespace cv;

static const char WINDOW[] = "Image window";

// PCL Definitions

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// PCL viewer //
//pcl::visualization::PCLVisualizer viewer("PCL Viewer");

enum { COLS = 640, ROWS = 480 };


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


int main (int argc, char** argv)
{
  ros::init(argc, argv, "cv_proj");
  //std::string input_file = "/home/igor/pcds/assembly_objs/ardrone_02_indoor.pcd";
  //std::string input_file = "/home/igor/pcds/assembly_objs/ardrone_03_outdoor.pcd";
  /*
  std::string input_file = "/home/igor/pcds/cluttered/3_objs_ardrone_indoor.pcd";
  std::string output_file = "/home/igor/pcds/cv_proj_out/out_cluttered_indoor_01.pcd";
  std::string template_file = "/home/igor/pcds/templates/indoor_template.pcd";
  std::string out_rgb = "/home/igor/pcds/cv_proj_out/out_result_05.jpg";
  */
  std::string input_file, out_pcd, template_file, out_rgb, out_transf_pcd;

  ros::param::param<std::string>("/cv_proj/input_file", input_file, "/home/igor/pcds/cluttered/3_objs_ardrone_indoor.pcd");
  //ros::param::param<std::string>("/cv_proj/out_pcd", out_pcd, "/home/igor/pcds/cv_proj_out/out_cluttered_indoor_01.pcd");
  ros::param::param<std::string>("/cv_proj/template_file", template_file, "/home/igor/pcds/templates/indoor_template.pcd");
  //ros::param::param<std::string>("/cv_proj/out_rgb", out_rgb, "/home/igor/pcds/cv_proj_out/out_result_05.jpg");
  


  boost::filesystem::path filepath(input_file);
  boost::filesystem::path filename = filepath.filename();
  filename = filename.stem(); // Get rid of the extension
  boost::filesystem::path dir = filepath.parent_path();

  std::string opencv_out_ext = "_filtered.png";
  std::string pcl_out_ext = "_filtered.pcd";
  std::string output_folder = "/output_cv_proj/";
  std::string output_stem;
  output_stem = dir.string() + output_folder + filename.string();

  out_rgb = output_stem + opencv_out_ext;
  out_pcd = output_stem + pcl_out_ext;
  out_transf_pcd = output_stem + "_templ" + pcl_out_ext;

  std::cout << out_rgb << std::endl;
  std::cout << out_pcd << std::endl;

  // Read in the cloud data
  pcl::PCDReader reader;
  PointCloudT::Ptr cloud (new PointCloudT), cloud_f (new PointCloudT);
  reader.read (input_file, *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  if (cloud->isOrganized()) {
    std::cout << "-- PointCloud cloud is organized" << std::endl;
    std::cout << "-- PointCloud cloud width: " << cloud->width << std::endl;
    std::cout << "-- PointCloud cloud height: " << cloud->height << std::endl;
  }

  /*
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<PointT> vg;
  PointCloudT::Ptr cloud_filtered (new PointCloudT);
  vg.setInputCloud (cloud);
  //vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.setLeafSize (0.001f, 0.001f, 0.001f);
  //
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
  */
  /**/
  PointCloudT::Ptr cloud_filtered (new PointCloudT);
  *cloud_filtered = *cloud;



  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  PointCloudT::Ptr cloud_plane (new PointCloudT ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    //extract.setUserFilterValue(999);
    extract.setKeepOrganized(true); 

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    //extract.filterDirectly(cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // To keep point cloud organized
    
    //extract.setKeepOrganized(true); 

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    //extract.filterDirectly(cloud_f);
    *cloud_filtered = *cloud_f;

    
  }

  cv::Mat result; 

  PC_to_Mat(cloud_filtered,result);

  imwrite( out_rgb, result );

  // For templates
  /*
  // Box filter
  pcl::PassThrough<PointT> pass_x;
  pass_x.setInputCloud (cloud_filtered);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (-0.2, 0.4);
  //pass.setFilterLimitsNegative (true);
  pass_x.filter (*cloud_filtered);

  pcl::PassThrough<PointT> pass_y;
  pass_y.setInputCloud (cloud_filtered);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-0.2, 0.4);
  //pass.setFilterLimitsNegative (true);
  pass_y.filter (*cloud_filtered);
  */

  // Save file
  pcl::io::savePCDFileASCII (out_pcd, *cloud_filtered);
  std::cerr << "Saved " << (*cloud_filtered).points.size () << " data points to PCD file." << std::endl;

  // Read in template file
  PointCloudT::Ptr cloud_template (new PointCloudT);
  reader.read (template_file, *cloud_template);

  pcl::VoxelGrid<PointT> vg_in;
  PointCloudT::Ptr cloud_filtered_vg (new PointCloudT);
  vg_in.setInputCloud (cloud_filtered);
  vg_in.setLeafSize (0.001f, 0.001f, 0.001f);  //vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg_in.filter (*cloud_filtered_vg);

  pcl::VoxelGrid<PointT> vg_temp;
  PointCloudT::Ptr cloud_template_vg (new PointCloudT);
  vg_temp.setInputCloud (cloud_template);
  vg_temp.setLeafSize (0.001f, 0.001f, 0.001f);  //vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg_temp.filter (*cloud_template_vg);

  // ICP
  
  PointCloudT::Ptr cloud_in (new PointCloudT), cloud_out (new PointCloudT);
  
  *cloud_in = *cloud_template_vg;
  *cloud_out = *cloud_filtered_vg;

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  PointCloudT Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  
  Eigen::Matrix4f transform = icp.getFinalTransformation ();
  std::cout << transform << std::endl;

  // Transform template to original frame
  PointCloudT::Ptr cloud_out2 (new PointCloudT);
  
  pcl::transformPointCloud(*cloud_in,*cloud_out2,transform);
  pcl::io::savePCDFileASCII (out_transf_pcd, *cloud_out2);
  std::cerr << "Saved " << (*cloud_out2).points.size () << " data points to PCD file." << std::endl;
  

  return (0);
}

/*
template <typename PointT> void
extractRGBFromPointCloud (PointCloudPtr input_cloud, pcl::PointCloud<pcl::RGB>::Ptr& output_cloud)
{
  // Extract RGB information from a point cloud and output the corresponding RGB point cloud  
  output_cloud->points.resize(input_cloud->height*input_cloud->width);
  output_cloud->width = input_cloud->width;
  output_cloud->height = input_cloud->height;

  pcl::RGB rgb_point;
  for (int j = 0; j < input_cloud->width; j++)
  {
    for (int i = 0; i < input_cloud->height; i++)
    { 
      rgb_point.r = (*input_cloud)(j,i).r;
      rgb_point.g = (*input_cloud)(j,i).g;
      rgb_point.b = (*input_cloud)(j,i).b;    
      (*output_cloud)(j,i) = rgb_point; 
    }
  }
}
*/