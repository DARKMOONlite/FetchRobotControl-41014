#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <thread>

const std::string TOPIC_NAME = "/detect_grasps/plot_grasps";

ros::Publisher individual_grasp_publisher;


void graspPoseCallback(const visualization_msgs::MarkerArrayConstPtr & _msg){
  // int size  = _msg->markers.size();
    ROS_INFO("Number of grasp poses found [%i]",_msg->markers.size());
    for(auto marker : _msg->markers){
        visualization_msgs::MarkerArray new_array;
        new_array.markers.push_back(marker);
        std_msgs::ColorRGBA colour;
        colour.a = 1;
        colour.r= 1;
        colour.g = 0.8;
        colour.b = 0.5;
        
        
        new_array.markers.front().color = colour;
        new_array.markers.front().pose.orientation;

      individual_grasp_publisher.publish(new_array);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      
    }

}

void setmarkercolour(visualization_msgs::Marker marker, std_msgs::ColorRGBA colour){
    
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "picker");

  ros::NodeHandle n;
  // std::this_thread::sleep_for(std::chrono::seconds(20));

  ROS_INFO("waiting for information on topic %s",TOPIC_NAME.c_str());
individual_grasp_publisher = n.advertise<visualization_msgs::MarkerArray>("/individual_grasps",1000);
  // ros::Subscriber sub = n.subscribe("/head_camera/depth_registered/points",1000,consensusPointCloudCallback);
  ros::Subscriber sub = n.subscribe(TOPIC_NAME, 1000, graspPoseCallback);
 
  ros::spin();

  ros::shutdown();
}