
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

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>


#include <scfms_helper.hpp>
// #include <pcl/point_cloud.h>


ros::Publisher output;
ros::Publisher box_removed;


//? taken from the Point cloud library example code http://pointclouds.org/documentation/tutorials/planar_segmentation.html#planar-segmentation



void consensusPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg){

    ROS_INFO("Point Cloud msg Received ");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::fromROSMsg<pcl::PointXYZ>(*msg,*cloud); //transfer from ros msg to pcl object
    ROS_INFO("cloud size [%i]",cloud->size());

    pcl::CropBox<pcl::PointXYZ> boxfilter; //create a box filter, which removes anything outside it

    boxfilter.setMin(Eigen::Vector4f(-5,0,0,1)); //set set box bounds
    boxfilter.setMax(Eigen::Vector4f(5,0.7,5,1));
    
    boxfilter.setInputCloud(cloud); //set the input cloud 
        std::vector<int> indicies;
    boxfilter.filter(indicies); // run the filter and get the output indicies that are within that box


    output.publish(pcl2roscloud(indicies,*cloud)); //publish those indicies


    std::vector<int> removed;
    boxfilter.setNegative(true); //run the inverse of the filter to get everything outside the box for visualisation
    boxfilter.filter(removed);

    box_removed.publish(pcl2roscloud(removed,*cloud)); 


    ROS_INFO("num remaining pts [%i]",indicies.size());
     ROS_INFO("num removed pts [%i]",removed.size());
    
}



//TODO update to accept topic name from launch argument
int main(int argc, char ** argv){



    ros::init(argc,argv,"/scfms/background_separator");

    ros::NodeHandle n;

    ROS_INFO("waiting for information on topic [/head_camera/depth_registered/points]");
    output = n.advertise<sensor_msgs::PointCloud2>(NS_SCENE+"/inv_background",1000);
    box_removed = n.advertise<sensor_msgs::PointCloud2>(NS_SCENE+"/background",1000);

    

    ros::Subscriber sub = n.subscribe("/head_camera/depth_registered/points",1000,consensusPointCloudCallback);

    ros::spin();

    ros::shutdown();
}