
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/filter_indices.h>

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