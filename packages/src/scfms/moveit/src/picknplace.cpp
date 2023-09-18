#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>




trajectory_msgs::JointTrajectory openGripper(trajectory_msgs::JointTrajectory& pose){
    pose.joint_names.resize(2);
    pose.joint_names.at(0) = "r_gripper_finger_joint";
    pose.joint_names.at(1)="l_gripper_finger_joint";
    

    pose.points.resize(1);
    pose.points.at(0).positions.resize(2);
    pose.points.at(0).positions.at(0)=0.05;
    pose.points.at(0).positions.at(1)=0.05;
    pose.points.at(0).time_from_start=ros::Duration(0.5);
    return(pose);
}

trajectory_msgs::JointTrajectory closeGripper(trajectory_msgs::JointTrajectory& pose){
    pose.joint_names.resize(2);
    pose.joint_names.at(0) = "r_gripper_finger_joint";
    pose.joint_names.at(1)="l_gripper_finger_joint";
    

    pose.points.resize(1);
    pose.points.at(0).positions.resize(2);
    pose.points.at(0).positions.at(0)=0.00;
    pose.points.at(0).positions.at(1)=0.00;
    pose.points.at(0).time_from_start=ros::Duration(0.5);
    return(pose);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group){
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1); //TODO UPDATE TO MULTIPLE GRASPS

    grasps[0].allowed_touch_objects.push_back("panda_hand_sc");

    // Setting grasp pose
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
    // of the cube). |br|
    // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
    // extra padding)
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.415;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.5;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasps[0].pre_grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick2
    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasps[0].grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick3
    // Set support surface as table1.
    move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);
}



int main(int argc, char** argv){
    ros::init(argc,argv);
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();



    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface arm_move_group("arm");
    moveit::planning_interface::MoveGroupInterface gripper_move_group("gripper");

    

}