
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <sensor_msgs/Image.h>





void cornerCallback(const sensor_msgs::ImageConstPtr& msg){
    ROS_INFO("Image msg Received ");
    Eigen::Matrix2Xi Image(msg->data,msg->step,msg->height);


    //Apply gausian blur 

    
}





//TODO update to accept topic name from launch argument
int main(int argc, char ** argv){


    ros::init(argc,argv,"corner_detection");

    ros::NodeHandle n;


    ros::Subscriber sub = n.subscribe("/head_camera/depth_downsample/image_raw",1000,cornerCallback);

    ros::spin();

    ros::shutdown();
}