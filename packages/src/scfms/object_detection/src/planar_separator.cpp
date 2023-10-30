
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

#include <pcl/common/distances.h>

// #include <pcl/point_cloud.h>
#include <chrono>
#include <thread>
ros::Publisher object_pub;
ros::Publisher table_pub;
ros::Publisher plane_pub;
ros::Publisher segmented_objects_pub, merged_objects_pub;
ros::Publisher gpd_pub;

sensor_msgs::PointCloud2 current_msg;
using PC = pcl::PointCloud<pcl::PointXYZ>;

static std::vector<colour> colours{{255, 102, 102}, {255, 178, 102}, {204, 255, 153}, {153, 255, 255}, {204, 153, 255}};

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
//   ROS_INFO("num inliers [%zd], num coefficients [%zd]",inliers->indices.size(),coefficients->values.size());
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

std::vector<PC> segmentation_growing(PC::Ptr segmentation_cloud)
{

  // pcl::copyPointCloud(*object_points, indicies_above_table, *segmentation_cloud);

  pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(segmentation_cloud);
  normal_estimator.setKSearch(50);
  normal_estimator.compute(*normals);

  pcl::IndicesPtr indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*segmentation_cloud, *indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize(50);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(30);
  reg.setInputCloud(segmentation_cloud);
  reg.setIndices(indices);
  reg.setInputNormals(normals);
  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  ROS_INFO("number of clusters [%zd]", clusters.size());
  std::stringstream ss;

  int i = 0;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> results;
  for (auto cluster : clusters)
  {

    ss << cluster.indices.size() << ", ";
    auto msg = pcl2roscloud(cluster.indices, *segmentation_cloud, colours.at(i % colours.size()));
    segmented_objects_pub.publish(msg);

    PC cloud;
    pcl::copyPointCloud(*segmentation_cloud, cluster.indices, cloud);
    results.push_back(cloud);

    i++;
  }
  ROS_INFO("size of clusters [%s]", ss.str().c_str());

  return (results);
}

std::vector<PC> CombinePC(std::vector<PC> clouds, float max_dist)
{
  std::vector<pcl::PointXYZ> averages;
  for (auto cloud : clouds)
  {
    averages.push_back(averagePointCloud(cloud));
  }
  std::vector<int> merge(clouds.size(), -1);

  for (int i = 0; i < clouds.size(); i++)
  {
    for (int j = i; j < clouds.size(); j++)
    {
      if (i != j)
      {
        if (pcl::euclideanDistance<pcl::PointXYZ, pcl::PointXYZ>(averages.at(i), averages.at(j)) < max_dist)
        {
          // only allow merging with 1 thing, not great but may work.
          merge.at(i) = j;
          break;
        }
      }
    }
  }
  std::vector<PC> merging_clouds(clouds.size());

  std::vector<bool> not_been_merged(clouds.size(), 0);
  for (int i = clouds.size() - 1; i >= 0; i--)
  {
    if (merge.at(i) == -1)
    {
      merging_clouds.at(i) = clouds.at(i);
      not_been_merged.at(i) = true;
    }
    else
    {
      merging_clouds.at(i) = clouds.at(i);
      merging_clouds.at(i) += merging_clouds.at(merge.at(i)); // merge the clouds
      not_been_merged.at(i) = true;
      not_been_merged.at(merge.at(i)) = false;
      // merge.at(i)=-1; // allow each point cloud to merge once.
    }
  }
  std::vector<PC> merged_clouds;

  for (int i = 0; i < clouds.size(); i++)
  {
    if (not_been_merged.at(i) == true)
    {
      merged_clouds.push_back(merging_clouds.at(i));
    }
  }
  return (merged_clouds);
}

void dataReadCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  if(msg->data.size()!=0){
    current_msg = *msg;
  }

}



void consensusUpdate(void)
{

  if(current_msg.data.size()==0){
    return;
  }
  auto msg_copy = current_msg;
  sensor_msgs::PointCloud2Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>(msg_copy);
  ROS_INFO("Point Cloud msg Received ");
  ros::Time start_time = ros::Time::now();

  PC::Ptr cloud(new PC());
  PC::Ptr final(new PC());

  pcl::fromROSMsg<pcl::PointXYZ>(*msg, *cloud); // load point cloud from ros msg
  ROS_INFO("cloud size [%zd]", cloud->size());

  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud)); // look for a plane

  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane); // create a ransac consensus looking for a plane

  ransac.setDistanceThreshold(.01);
  ransac.computeModel();
  std::vector<int> inliers;

  ransac.getInliers(inliers); // get the inliers within .01 of that plane

  auto outliers = getInverseIndicies(inliers, *cloud); // gets the outliers

  plane_pub.publish(pcl2roscloud(inliers, *cloud));

  ros::Time end_time = ros::Time::now();                                            // get the end time
  ros::Duration duration = end_time - start_time;                                   // calculate the duration
  ROS_INFO("Plane Random Consensus took %f seconds to complete", duration.toSec()); // print the duration

  //? ------------------- split objects above and below table -------------------------------------
  start_time = ros::Time::now();
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

  table_pub.publish(pcl2roscloud(indicies_below_table, *object_points));

  auto indicies_above_table = getInverseIndicies(indicies_below_table, *object_points);

  object_pub.publish(pcl2roscloud(indicies_above_table, *object_points));

  end_time = ros::Time::now();                                                        // get the end time
  duration = end_time - start_time;                                                   // calculate the duration
  ROS_INFO("objects split by plane took [%f] seconds to complete", duration.toSec()); // print the duration

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
  start_time = ros::Time::now();

  PC::Ptr obj_above_table_cloud(new PC);
  pcl::copyPointCloud(*object_points, indicies_above_table, *obj_above_table_cloud);

  auto vector_clouds = segmentation_growing(obj_above_table_cloud);

  //? --------------------- Merge clouds based on proximity of centroids --------------------

  auto merged_objects = CombinePC(vector_clouds, 0.2);
  // sort array, largest to smallest to make colours consistent
  std::sort(merged_objects.begin(), merged_objects.end(), [](PC &a, PC &b)
            { return (a.points.size() > b.points.size()); });

  ROS_INFO("  num merged clouds [%zd]", merged_objects.size());

  std::stringstream ss;
  int i = 0;
  for (auto merged_clouds : merged_objects)
  {
    i++;
    ss << merged_clouds.size() << " ,";
    // merged_objects_pub.publish(pcl2roscloud(merged_clouds,colours.at(i%colours.size())));
  }
  ROS_INFO("  merged sizes [%s]", ss.str().c_str());

  end_time = ros::Time::now();                                                    // get the end time
  duration = end_time - start_time;                                               // calculate the duration
  ROS_INFO("cloud segmentation and merging took [%f] seconds", duration.toSec()); // print the duration

  //? --------------------- Convert to Index Cloud Msg for more effective GPD use;

  for (auto merged_clouds : merged_objects)
  {
    std::shared_ptr<PC> cloud_shared_ptr = std::make_shared<PC>(merged_clouds);
    std::vector<int> reframed_merged_indicies = reframeIndicies(cloud_shared_ptr, cloud);
    auto gpd_msg = PointCloud2GPDIndexCloud(reframed_merged_indicies, cloud);
    gpd_pub.publish(gpd_msg);

    merged_objects_pub.publish(pcl2roscloud(reframed_merged_indicies, *cloud));
  }
}

// TODO update to accept topic name from launch argument
int main(int argc, char **argv)
{

  ros::init(argc, argv, "scfms/planar_separator");

  ros::NodeHandle n;
  // std::this_thread::sleep_for(std::chrono::seconds(20));

  // ROS_INFO("waiting for information on topic [/removed_background]");
  object_pub = n.advertise<sensor_msgs::PointCloud2>(NS_SCENE + "/objects", 1);
  table_pub = n.advertise<sensor_msgs::PointCloud2>(NS_SCENE + "/table_legs", 1);
  plane_pub = n.advertise<sensor_msgs::PointCloud2>(NS_SCENE + "/plane", 1);
  segmented_objects_pub = n.advertise<sensor_msgs::PointCloud2>(NS_OBJECTS + "/segments", 1);
  merged_objects_pub = n.advertise<sensor_msgs::PointCloud2>(NS_OBJECTS + "/merged", 1);
  gpd_pub = n.advertise<gpd_ros::CloudIndexed>(NS_OBJECTS + "/gpd_indexed", 1);
  // ros::Subscriber sub = n.subscribe("/head_camera/depth_registered/points",1000,consensusUpdate);
  ros::Subscriber sub = n.subscribe(NS_SCENE + "/inv_background", 1, dataReadCallback);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok())
  {
    consensusUpdate();
  }

  ros::shutdown();
}