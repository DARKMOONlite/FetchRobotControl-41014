
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
#include <cstdint>
#include <optional>
#include <pcl/search/kdtree.h>
#include <gpd_ros/CloudIndexed.h>

struct colour{
    uint8_t r=255;
    uint8_t g=255;
    uint8_t b=255;

    colour(uint8_t _r, uint8_t _g, uint8_t _b){
        r=_r;
        g=_g;
        b=_b;
    }
};


/**
 * @brief namespaces for publishing and subscribing
 * 
 */
const std::string NS_SCENE = "/scfms_scene", NS_OBJECTS = "/scfms_objects";



sensor_msgs::PointCloud2 pcl2roscloud(pcl::PointCloud<pcl::PointXYZ> cloud, colour _colour = (colour){255,255,255}){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::copyPointCloud(cloud,*final);
    // ROS_WARN("colour values [%i %i %i]",_colour.r,_colour.g,_colour.b);
    // for(auto point: *final){
    //     point.r = _colour.r;
    //     point.g = _colour.b;
    //     point.b = _colour.b;
    //     point.a = 0.5;
    //     // point.rgb = (static_cast<uint32_t>(_colour.r) << 16 | 
    //     //             static_cast<uint32_t>(_colour.g) << 8 | 
    //     //             static_cast<uint32_t>(_colour.b));

    // }
    int32_t rgb = (static_cast<uint32_t>(_colour.r) << 16 |
                    static_cast<uint32_t>(_colour.g) << 8 |
                     static_cast<uint32_t>(_colour.b));
    for (auto &point : final->points){
        point.rgb=rgb;
    }
    
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*final,output);


    return(output);
}

sensor_msgs::PointCloud2 pcl2roscloud(std::vector<int> indicies, pcl::PointCloud<pcl::PointXYZ> cloud,colour _colour = (colour){255,255,255} ){
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(cloud,indicies,*final);

    return(pcl2roscloud(*final,_colour));
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
std::vector<pcl::PointCloud<pcl::PointXYZ>> ransacAllModels(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int min_model_size = 200, const int search_depth_percentage = 5){
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud,*new_cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZ>> results;
    
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
        
        

         //extract found indicies from the point cloud 
        extract.setInputCloud(new_cloud);

        ROS_INFO("ransac function cloud size [%i]",new_cloud->size());

        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*new_cloud);
        extract.setNegative(false);
        results.push_back(pcl::PointCloud<pcl::PointXYZ>());
        
        extract.filter(results.back());
        
        ROS_INFO("inliers being removed [%i]",inliers->size());


 
    }
    return(results);

}


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
/**
 * @brief finds the points represented by the index extracted from the indexed_cloud in the total cloud;
 * 
 * @param indexed_cloud 
 * @param total_cloud 
 * @return std::vector<int> returns the inputed indicies but in the frame of @param total_cloud
 */
std::vector<int> reframeIndicies(pcl::PointCloud<pcl::PointXYZ>::Ptr indexed_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr total_cloud, std::optional<std::vector<int>> index = std::nullopt,double max_warn_dist = 0.5){
    pcl::KdTreeFLANN<pcl::PointXYZ> kdt;
    std::vector<int> new_index;
    kdt.setInputCloud(total_cloud);
    if(index){
        for(auto point : *index){
            std::vector<int> output_index;
            std::vector<float> pose_delta;
            kdt.nearestKSearch(indexed_cloud->at(point),1,output_index,pose_delta);
            new_index.push_back(output_index.front());
            if(pose_delta.front()>max_warn_dist){
               ROS_INFO("when reframing indicies, 1 point was found to be > %f away from its expected point",pose_delta.front());
            } 
        }
    }
    else{
        for(auto point : indexed_cloud->points){
            std::vector<int> output_index;
            std::vector<float> pose_delta;
            kdt.nearestKSearch(point,1,output_index,pose_delta);
            new_index.push_back(output_index.front());
            if(pose_delta.front()>max_warn_dist){
               ROS_INFO("when reframing indicies, 1 point was found to be > %f away from its expected point",pose_delta.front());
            } 

    }
    }


    return(new_index);

    
    
}
/**
 * @brief IDK doesn't work as intended, grabs the table legs which is not what i want.
 * 
 * @param indicies 
 * @param cloud 
 * @return gpd_ros::CloudIndexed 
 */
gpd_ros::CloudIndexed PointCloud2GPDIndexCloud(std::vector<int> indicies,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ){
gpd_ros::CloudIndexed output;
for(auto index : indicies){
    std_msgs::Int64 val;
    val.data = index;
    output.indices.push_back(val);
}
output.cloud_sources.cloud =pcl2roscloud(*cloud);
    std_msgs::Int64 camera_source;
    camera_source.data=0;
    geometry_msgs::Point view_point;
    view_point.x=0;
    view_point.y=0;
    view_point.z=0;
for(auto point : *cloud){

output.cloud_sources.camera_source.push_back(camera_source);

}
output.cloud_sources.view_points.push_back(view_point);

return(output);
}


