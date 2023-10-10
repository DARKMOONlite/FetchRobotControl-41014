
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

#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>

// #include <pcl/point_cloud.h>


ros::Publisher output;

//? taken from the Point cloud library example code http://pointclouds.org/documentation/tutorials/planar_segmentation.html#planar-segmentation

void segmentPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg){
     ROS_INFO("Point Cloud msg Received ");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
   
    pcl::fromROSMsg<pcl::PointXYZ>(*msg,*cloud);
    ROS_INFO("cloud size [%i]",cloud->size());
    
    // auto temp = *cloud;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);


  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  seg.setMaxIterations(100);

  // Mandatory
  
  seg.setModelType (pcl::SACMODEL_PLANE); //set the model
  seg.setMethodType (pcl::SAC_RANSAC); // set the method
  seg.setDistanceThreshold (0.05); //set distance threshold for inlier


  seg.setInputCloud(cloud); 
  seg.segment (*inliers, *coefficients); // setment the point cloud
  ROS_INFO("num inliers [%i], num coefficients [%i]",inliers->indices.size(),coefficients->values.size());
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

void consensusPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg){


    ROS_INFO("Point Cloud msg Received ");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::fromROSMsg<pcl::PointXYZ>(*msg,*cloud);
    ROS_INFO("cloud size [%i]",cloud->size());

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ROS_INFO("setting ransac values");
    ransac.setDistanceThreshold(.01);
    ransac.computeModel();
    std::vector<int> inliers;
    ransac.getInliers(inliers);
    ROS_INFO("getting inliers");

    pcl::copyPointCloud(*cloud,inliers,*final);
    sensor_msgs::PointCloud2 segmentedPointCloud;
    pcl::toROSMsg(*final,segmentedPointCloud);
    output.publish(segmentedPointCloud);
      //  ROS_INFO("inliers:");
  // for (const auto& idx: inliers){
  //   ROS_INFO(" [%i]: [%f, %f, %f] \n",idx,cloud->points[idx].x,cloud->points[idx].y,cloud->points[idx].z);  
  //   }




   

    
}



//TODO update to accept topic name from launch argument
int main(int argc, char ** argv){

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // // Fill in the cloud data
  // cloud->width  = 15;
  // cloud->height = 1;
  // cloud->points.resize (cloud->width * cloud->height);

  // // Generate the data
  // for (auto& point: *cloud)
  // {
  //   point.x = 1024 * rand () / (RAND_MAX + 1.0f);
  //   point.y = 1024 * rand () / (RAND_MAX + 1.0f);
  //   point.z = 1.0;
  // }

  // // Set a few outliers
  // (*cloud)[0].z = 2.0;
  // (*cloud)[3].z = -2.0;
  // (*cloud)[6].z = 4.0;

  // std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;
  // for (const auto& point: *cloud)
  //   std::cerr << "    " << point.x << " "
  //                       << point.y << " "
  //                       << point.z << std::endl;

  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // // Create the segmentation object
  // pcl::SACSegmentation<pcl::PointXYZ> seg;
  // // Optional
  // seg.setOptimizeCoefficients (true);
  // // Mandatory
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setDistanceThreshold (0.01);

  // seg.setInputCloud (cloud);
  // seg.segment (*inliers, *coefficients);

  // if (inliers->indices.size () == 0)
  // {
  //   PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
  //   return (-1);
  // }

  // std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
  //                                     << coefficients->values[1] << " "
  //                                     << coefficients->values[2] << " " 
  //                                     << coefficients->values[3] << std::endl;

  // std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  // for (const auto& idx: inliers->indices)
  //   std::cerr << idx << "    " << cloud->points[idx].x << " "
  //                              << cloud->points[idx].y << " "
  //                              << cloud->points[idx].z << std::endl;

  // return (0);

    ros::init(argc,argv,"corner_detection");

    ros::NodeHandle n;

    ROS_INFO("waiting for information on topic [/head_camera/depth_registered/points]");
    output = n.advertise<sensor_msgs::PointCloud2>("/planar_seperator",1000);
    ros::Subscriber sub = n.subscribe("/head_camera/depth_registered/points",1000,consensusPointCloudCallback);

    ros::spin();

    ros::shutdown();
}