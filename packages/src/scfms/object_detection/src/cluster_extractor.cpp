
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>



#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "scfms_helper.hpp"


ros::Publisher clusters;

void clusterExtractionCallback(const sensor_msgs::PointCloud2ConstPtr & _msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>());



    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg<pcl::PointXYZ>(*_msg,*cloud);
    ROS_INFO("cloud size [%i]",cloud->size());


//     pcl::VoxelGrid<pcl::PointXYZ> vg;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//     vg.setInputCloud (cloud);
//     vg.setLeafSize (0.01f, 0.01f, 0.01f);
//     vg.filter (*cloud_filtered);    
//     ROS_INFO("after filtering, size is [%i]",cloud_filtered->size());


//     pcl::SACSegmentation<pcl::PointXYZ> seg;
//     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
// //    pcl::PCDWriter writer;
//    seg.setOptimizeCoefficients (true);
//    seg.setModelType (pcl::SACMODEL_SPHERE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setMaxIterations (100);
   
//    seg.setDistanceThreshold (0.05);
//    seg.setRadiusLimits(0,0.5);
   



//    int nr_points = (int) cloud_filtered->size ();
//    while (cloud_filtered->size () > 0.3 * nr_points)
//    {
//         // Segment the largest planar component from the remaining cloud
//         seg.setInputCloud (cloud_filtered);
//         seg.segment (*inliers, *coefficients);

//         if (inliers->indices.size () == 0){
//             std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//             break;
//         }

//         // Extract the planar inliers from the input cloud
//         pcl::ExtractIndices<pcl::PointXYZ> extract;
//         extract.setInputCloud (cloud_filtered);
//         extract.setIndices (inliers);
//         extract.setNegative (false);

//         // Get the points associated with the planar surface
//         extract.filter (*cloud_plane);
//         std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

//         // Remove the planar inliers, extract the rest
//         extract.setNegative (true);
//         extract.filter (*cloud_f);
//         *cloud_filtered = *cloud_f;
//    }


//     // Creating the KdTree object for the search method of the extraction
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//   tree->setInputCloud (cloud_filtered);

//   std::vector<pcl::PointIndices> cluster_indices;
//   pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//   ec.setClusterTolerance (0.02); // 2cm
//   ec.setMinClusterSize (100);
//   ec.setMaxClusterSize (25000);
//   ec.setSearchMethod (tree);
//   ec.setInputCloud (cloud_filtered);
//   ec.extract (cluster_indices);


//   for (const auto& cluster : cluster_indices)
//   {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//     for (const auto& idx : cluster.indices) {
//       cloud_cluster->push_back((*cloud_filtered)[idx]);
//     } //*
//     cloud_cluster->width = cloud_cluster->size ();
//     cloud_cluster->height = 1;
//     cloud_cluster->is_dense = true;

//     clusters.publish(extract2ros(cluster.indices,*cloud_filtered));
//     // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
//     // std::stringstream ss;
//     // ss << std::setw(4) << std::setfill('0') << j;
//     // writer.write<pcl::PointXYZ> ("cloud_cluster_" + ss.str () + ".pcd", *cloud_cluster, false); //*

//   }


}



const std::string sub_topic = "/objects";


int main(int argc, char** argv){
        ros::init(argc,argv,"background");

    ros::NodeHandle n;

    ROS_INFO("waiting for information on topic [%s]",sub_topic.c_str());
    clusters = n.advertise<sensor_msgs::PointCloud2>("/object_clusters",1000);
    // box_removed = n.advertise<sensor_msgs::PointCloud2>("/background",1000);

    

    ros::Subscriber sub = n.subscribe(sub_topic,1000,clusterExtractionCallback);

    ros::spin();

    ros::shutdown();
}