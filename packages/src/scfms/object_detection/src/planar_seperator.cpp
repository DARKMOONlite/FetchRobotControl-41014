
#include <ros/ros.h>


#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>


//? taken from the Point cloud library example code http://pointclouds.org/documentation/tutorials/planar_segmentation.html#planar-segmentation

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg){
     ROS_INFO("Point Cloud msg Received ");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
   
    pcl::fromROSMsg<pcl::PointXYZ>(*msg,*cloud);
    // auto temp = *cloud;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (1);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    return;
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (const auto& idx: inliers->indices)
    std::cerr << idx << "    " << cloud->points[idx].x << " "
                               << cloud->points[idx].y << " "
                               << cloud->points[idx].z << std::endl;

  return;
}





//TODO update to accept topic name from launch argument
int main(int argc, char ** argv){


    ros::init(argc,argv,"corner_detection");

    ros::NodeHandle n;


    ros::Subscriber sub = n.subscribe("/head_camera/depth_registered/points",1000,pointCloudCallback);

    ros::spin();

    ros::shutdown();
}