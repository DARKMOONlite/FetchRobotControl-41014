
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
#include <pcl/filters/extract_indices.h>

#include <od_helper.hpp>


#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

// #include <pcl/point_cloud.h>


ros::Publisher output;
ros::Publisher output_below;
ros::Publisher plane;

//? taken from the Point cloud library example code http://pointclouds.org/documentation/tutorials/planar_segmentation.html#planar-segmentation

// void segmentPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg){
//      ROS_INFO("Point Cloud msg Received ");
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
   
//     pcl::fromROSMsg<pcl::PointXYZ>(*msg,*cloud);
//     ROS_INFO("cloud size [%i]",cloud->size());
    
//     // auto temp = *cloud;
//     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);


//   // Create the segmentation object
//   pcl::SACSegmentation<pcl::PointXYZ> seg;
//   // Optional
//   seg.setOptimizeCoefficients (true);
//   seg.setMaxIterations(100);

//   // Mandatory
  
//   seg.setModelType (pcl::SACMODEL_PLANE); //set the model
//   seg.setMethodType (pcl::SAC_RANSAC); // set the method
//   seg.setDistanceThreshold (0.05); //set distance threshold for inlier


//   seg.setInputCloud(cloud); 
//   seg.segment (*inliers, *coefficients); // setment the point cloud
//   ROS_INFO("num inliers [%i], num coefficients [%i]",inliers->indices.size(),coefficients->values.size());
//   if (inliers->indices.size () == 0)
//   {
//     PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
//     return;
//   }

//   std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
//                                       << coefficients->values[1] << " "
//                                       << coefficients->values[2] << " " 
//                                       << coefficients->values[3] << std::endl;

//   std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
//   for (const auto& idx: inliers->indices)
//     std::cerr << idx << "    " << cloud->points[idx].x << " "
//                                << cloud->points[idx].y << " "
//                                << cloud->points[idx].z << std::endl;

//   return;
// }

pcl::PointXYZ averagePointCloud(pcl::PointCloud<pcl::PointXYZ> cloud){
  size_t i=0;
  pcl::PointXYZ output;
  for(auto point : cloud){
    i++;
    output.x+= point.x;
    output.y+= point.y;
    output.z+= point.z;
    
  }

  output.x= output.x/i;
  output.y= output.y/i;
  output.z= output.z/i;

 return(output);

}


void consensusPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg){


    ROS_INFO("Point Cloud msg Received ");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::fromROSMsg<pcl::PointXYZ>(*msg,*cloud); // load point cloud from ros msg
    ROS_INFO("cloud size [%i]",cloud->size());

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud)); // look for a plane

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_plane); // create a ransac consensus looking for a plane

    ROS_INFO("setting ransac values");
    ransac.setDistanceThreshold(.01);
    ransac.computeModel();
    std::vector<int> inliers;

    ransac.getInliers(inliers); //get the inliers within .01 of that plane


  

  auto outliers = getInverseIndicies(inliers,*cloud);//gets the outliers

  // pcl::ExtractIndices<pcl::PointXYZ> extract;

  // extract.setInputCloud(cloud);
  
//   pcl::PointIndices::Ptr temp(new pcl::PointIndices);
// //  pcl::IndicesPtr temp(new pcl::IndicesPtr());
//   temp->indices = inliers;
//   extract.setIndices(temp);
//   extract.setNegative(true);
//   std::vector<int> outliers;
//   extract.filter(outliers);


     ROS_INFO("num remaining pts [%i]",inliers.size());
     ROS_INFO("num removed pts [%i]",outliers.size());


    plane.publish(extract2ros(inliers,*cloud));


// new clouds for removing all objects found below the table plane.


  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud,inliers,*plane_points);

  pcl::PointCloud<pcl::PointXYZ>::Ptr object_points(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud,outliers,*object_points);

  auto plane_point = averagePointCloud(*plane_points);

 
  ROS_INFO("height of table [%f]",plane_point.y);

  
  pcl::CropBox<pcl::PointXYZ> boxfilter;
  boxfilter.setMin(Eigen::Vector4f( -5,0,-5,1));
  boxfilter.setMax(Eigen::Vector4f( 5,plane_point.y,5,1));
  boxfilter.setNegative(true);

  boxfilter.setInputCloud(object_points);
  std::vector<int> indicies_below_table;
  boxfilter.filter(indicies_below_table);

  output_below.publish(extract2ros(indicies_below_table,*object_points));
  
  auto indicies_above_table = getInverseIndicies(indicies_below_table,*object_points);

  // boxfilter.setNegative(false);
  // std::vector<int> indicies_above_table;
  // boxfilter.filter(indicies_above_table);

    output.publish(extract2ros(indicies_above_table,*object_points));
    
}



//TODO update to accept topic name from launch argument
int main(int argc, char ** argv){

    ros::init(argc,argv,"planar");

    ros::NodeHandle n;

    ROS_INFO("waiting for information on topic [/removed_background]");
    output = n.advertise<sensor_msgs::PointCloud2>("/objects",1000);
    output_below = n.advertise<sensor_msgs::PointCloud2>("/table_legs",1000);
    plane = n.advertise<sensor_msgs::PointCloud2>("/plane",1000);
    // ros::Subscriber sub = n.subscribe("/head_camera/depth_registered/points",1000,consensusPointCloudCallback);
       ros::Subscriber sub = n.subscribe("/removed_background",1000,consensusPointCloudCallback);


    ros::spin();

    ros::shutdown();
}