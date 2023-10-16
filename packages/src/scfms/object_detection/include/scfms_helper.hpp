
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




// ,
template <typename T>
std::vector<std::vector<int>> ransacAllPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int min_model_size = 200, const int search_depth_percentage = 5){
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud,*new_cloud);

    std::vector<std::vector<int>> results;
    
    pcl::IndicesPtr inliers (new std::vector<int>);



    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    while(new_cloud->size() > cloud->size()*5/100){

        pcl::SampleConsensusModel<pcl::PointXYZ>::Ptr new_model(new T(new_cloud)); //! find out why the constructor needs to be called each loop instead of just setinputcloud on the changed cloud
        new_model->setInputCloud(new_cloud);

        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(new_model);
        ransac.setDistanceThreshold(.01);
        ransac.computeModel();
        ransac.getInliers(*inliers);
        if(inliers->size() <min_model_size){ // if no models that match are found then exit
            break;
        }
        results.push_back(*inliers);
        

         //extract found indicies from the point cloud 
        extract.setInputCloud(new_cloud);

        ROS_INFO("ransac function cloud size [%i]",new_cloud->size());

        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*new_cloud);
        
        ROS_INFO("inliers being removed [%i]",inliers->size());


 
    }
    return(results);

}

