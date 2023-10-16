
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

#include <scfms_helper.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>



// #include <pcl/point_cloud.h>
#include <chrono>

ros::Publisher object_pub;
ros::Publisher table_pub;
ros::Publisher plane_pub;
ros::Publisher segmented_objects , seg_2, seg_3, seg_4;

using PC = pcl::PointCloud<pcl::PointXYZ> ;

//? taken from the Point cloud library example code http://pointclouds.org/documentation/tutorials/planar_segmentation.html#planar-segmentation

// void segmentPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg){
//      ROS_INFO("Point Cloud msg Received ");
//     PC::Ptr cloud(new PC());

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


void segmentation_growing(PC::Ptr segmentation_cloud){

  // pcl::copyPointCloud(*object_points, indicies_above_table, *segmentation_cloud);

  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (segmentation_cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::removeNaNFromPointCloud(*segmentation_cloud, *indices);

 pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (segmentation_cloud);
  reg.setIndices (indices);
  reg.setInputNormals (normals);
  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  ROS_INFO("number of clusters [%i]",clusters.size());
  std::stringstream ss;
  for(auto cluster : clusters){
    ss <<cluster.indices.size() << ", ";
    auto msg = extract2ros(cluster.indices,*segmentation_cloud);
    segmented_objects.publish(msg);
  }
  ROS_INFO("size of clusters [%s]",ss.str().c_str());
  
}


void consensusPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{

  ROS_INFO("Point Cloud msg Received ");
  PC::Ptr cloud(new PC());
  PC::Ptr final(new PC());

  pcl::fromROSMsg<pcl::PointXYZ>(*msg, *cloud); // load point cloud from ros msg
  ROS_INFO("cloud size [%i]", cloud->size());

  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud)); // look for a plane

  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane); // create a ransac consensus looking for a plane

  ROS_INFO("setting ransac values");
  ransac.setDistanceThreshold(.01);
  ransac.computeModel();
  std::vector<int> inliers;

  ransac.getInliers(inliers); // get the inliers within .01 of that plane

  auto outliers = getInverseIndicies(inliers, *cloud); // gets the outliers

  // pcl::ExtractIndices<pcl::PointXYZ> extract;

  // extract.setInputCloud(cloud);

  //   pcl::PointIndices::Ptr temp(new pcl::PointIndices);
  // //  pcl::IndicesPtr temp(new pcl::IndicesPtr());
  //   temp->indices = inliers;
  //   extract.setIndices(temp);
  //   extract.setNegative(true);
  //   std::vector<int> outliers;
  //   extract.filter(outliers);

  ROS_INFO("num remaining pts [%i]", inliers.size());
  ROS_INFO("num removed pts [%i]", outliers.size());

  plane_pub.publish(extract2ros(inliers, *cloud));

  //? ------------------- split objects above and below table -------------------------------------

  PC::Ptr plane_points(new PC());
  pcl::copyPointCloud(*cloud, inliers, *plane_points);

  PC::Ptr object_points(new PC());
  pcl::copyPointCloud(*cloud, outliers, *object_points);

  auto plane_point = averagePointCloud(*plane_points);

  ROS_INFO("height of table [%f]", plane_point.y);

  pcl::CropBox<pcl::PointXYZ> boxfilter;
  boxfilter.setMin(Eigen::Vector4f(-5, 0, -5, 1));
  boxfilter.setMax(Eigen::Vector4f(5, plane_point.y - 0.01, 5, 1)); // 0.01 border to remove erotius plane points not caught in plane
  boxfilter.setNegative(true);

  boxfilter.setInputCloud(object_points);
  std::vector<int> indicies_below_table;
  boxfilter.filter(indicies_below_table);

  table_pub.publish(extract2ros(indicies_below_table, *object_points));

  auto indicies_above_table = getInverseIndicies(indicies_below_table, *object_points);


  object_pub.publish(extract2ros(indicies_above_table, *object_points));




//? ------------------------------- trying to use ransac to find planes ------

  // PC::Ptr obj_pc(new PC());
  // pcl::copyPointCloud(*object_points, indicies_above_table, *obj_pc);

  // auto object_planes = ransacAllModels<pcl::SampleConsensusModelPlane<pcl::PointXYZ>>(obj_pc);
  // for (auto pc : object_planes)
  // {
  //   sensor_msgs::PointCloud2 output;
  //   pcl::toROSMsg(pc, output);
  //   segmented_objects.publish(output);
  // }
  // auto time = std::chrono::system_clock::now();

  // ROS_INFO("number of planes found [%i]", object_planes.size());
  // std::stringstream ss;
  // for (auto value : object_planes)
  // {
  //   ss << value.size() << ", ";
  // }
  // ROS_INFO("number of points in each plane [%s]", ss.str().c_str());
  // ROS_INFO("time taken to find all planes, %f", std::chrono::duration<double>(std::chrono::system_clock::now() - time).count());
//? ------- Region Growing Segmentation to seperate distinct objects ----------------------------
PC::Ptr obj_above_table_cloud(new PC);
pcl::copyPointCloud(*object_points,indicies_above_table,*obj_above_table_cloud);

segmentation_growing(obj_above_table_cloud);

}

// TODO update to accept topic name from launch argument
int main(int argc, char **argv)
{

  ros::init(argc, argv, "planar");

  ros::NodeHandle n;

  ROS_INFO("waiting for information on topic [/removed_background]");
  object_pub = n.advertise<sensor_msgs::PointCloud2>("/objects", 1000);
  table_pub = n.advertise<sensor_msgs::PointCloud2>("/table_legs", 1000);
  plane_pub = n.advertise<sensor_msgs::PointCloud2>("/plane", 1000);
  segmented_objects = n.advertise<sensor_msgs::PointCloud2>("/found_planes", 1000);
  seg_2 = n.advertise<sensor_msgs::PointCloud2>("/seg2", 1000);
    seg_3 = n.advertise<sensor_msgs::PointCloud2>("/seg3", 1000);
      seg_4 = n.advertise<sensor_msgs::PointCloud2>("/seg4", 1000);
  // ros::Subscriber sub = n.subscribe("/head_camera/depth_registered/points",1000,consensusPointCloudCallback);
  ros::Subscriber sub = n.subscribe("/removed_background", 1000, consensusPointCloudCallback);
 
  ros::spin();

  ros::shutdown();
}