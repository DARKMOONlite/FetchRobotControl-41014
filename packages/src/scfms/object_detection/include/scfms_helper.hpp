
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/filter_indices.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/extract_indices.h>
sensor_msgs::PointCloud2 extract2ros(std::vector<int> indicies, pcl::PointCloud<pcl::PointXYZ> cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(cloud,indicies,*final);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*final,output);
    return(output);
}

/**
 * @brief returns the inverse of the indicies. i.e. the indicies that did not meet the filter / consensus
 * 
 * @param indicies 
 * @param cloud 
 * @return std::vector<int> 
 */
std::vector<int> getInverseIndicies(std::vector<int>& indicies, pcl::PointCloud<pcl::PointXYZ>&  cloud){
    std::vector<int> output;
    for(int i=0;i<cloud.size();i++){
        if(std::find(indicies.begin(),indicies.end(),i)==indicies.end()){
            output.push_back(i);
        }
    }
return(output);
}




// pcl::SampleConsensusModel<pcl::PointXYZ>::ConstPtr model,
std::vector<std::vector<int>> ransacAllPlanes( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud,*new_cloud);

 
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(new_cloud)); // look for a plane

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);

    ransac.setDistanceThreshold(.01);

    std::vector<std::vector<int>> results;
   
    pcl::IndicesPtr inliers (new std::vector<int>);
    // inliersPtr->data = inliers;

    while(new_cloud->size()>0){
        ransac.computeModel();
        ransac.getInliers(*inliers);
        if(inliers->size()==0){ // if no models that match are found then exit
            break;
        }
        results.push_back(*inliers);

        pcl::ExtractIndices<pcl::PointXYZ> extract; //extract found indicies from the point cloud 
        extract.setInputCloud(new_cloud);

        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*new_cloud);


 
    }
    return(results);

}

