#!/usr/bin/env python

import copy
import actionlib
import rospy
import time
from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
import moveit_commander
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal, GripperCommandAction, GripperCommandGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes, Grasp
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray
from gpd_ros.msg import GraspConfigList

import tf2_ros, tf2_geometry_msgs

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo("...connected.")

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)

        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        if self.client.wait_for_result()  ==True: 
            rospy.loginfo("Arrived")
        else:
            rospy.loginfo("Failed to reach desitnation")

# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()
        rospy.loginfo("...connected.")

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

class GripperClient(object):

    def __init__(self):
        rospy.loginfo("Waiting for gripper_controller...")
        self.client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        self.client.wait_for_server()
        rospy.loginfo("...connected.")

        self.max = 0.1
        self.min = 0.0

    def open(self, wait=True, timeout=5.0):
        self.move(0.1, 10.0, wait=wait, timeout=timeout)

    def close(self, wait=True, timeout=5.0):
        self.move(0.0, 10.0, wait=wait, timeout=timeout)

    def move(self, position, max_effort=10.0, wait=True, timeout=5.0):
        if position > self.max or position < self.min:
            rospy.logerr("Gripper position must be between {} and {}".format(self.min, self.max))
            return False
        goal = GripperCommandGoal()
        goal.command.max_effort= max_effort
        goal.command.position = position
        self.client.send_goal(goal)
        if wait:
            self.client.wait_for_result(rospy.Duration(timeout))

        return self.client.get_result()



import numpy as np
from scipy.spatial.transform import Rotation
from transforms3d import quaternions
# def grab_object(Pose,size):


# def rot2quat(R):

#     r = Rotation.from_dcm(R)
#     quat = r.as_quat()
#     return quat




# def grasp_callback(data):
#     grasp_callback.counter += 1
#     grasps = data.grasps
#     rospy.loginfo("Number of grasps received: {}".format(len(grasps)))

#     rospy.loginfo("going to attempt to grasp in order")
#     output_grasps = []
#     for grasp in grasps:
#         ee_pose = PoseStamped() 
        

#         ee_pose.pose.position.x = grasp.position.x
#         ee_pose.pose.position.y = grasp.position.y
#         ee_pose.pose.position.z = grasp.position.z
       
#         # rotation = np.array([[grasp.axis.x,grasp.axis.y,grasp.axis.z],[grasp.binormal.x,grasp.binormal.y,grasp.binormal.z],[grasp.approach.x,grasp.approach.y,grasp.approach.z]])
#         rotation = np.array([[grasp.approach.x,grasp.binormal.x,grasp.axis.x],[grasp.approach.y,grasp.binormal.y,grasp.axis.y],[grasp.approach.z,grasp.binormal.z,grasp.axis.z]])
#         rotate_around_y = np.array([[0,0,1],[0,1,0],[-1,0,0]])
#         rotate_around_z = np.array([[0,-1,0],[1,0,0],[0,0,1]])
#         rotation_adjusted = np.matmul(np.matmul(rotate_around_y,rotate_around_z),rotation)
#         # rotation*rotate_around_y*rotate_around_z
#         quaternion = quaternions.mat2quat(rotation)
#         rospy.loginfo("matrix2 :{}".format(np.matmul(rotation,rotate_around_y)))
#         rospy.loginfo("matrix3 : {}".format(rotation_adjusted))
#         # rospy.loginfo("matrix2: {}".format(Rotation.fr))
#         ee_pose.pose.orientation.x = quaternion[0]
#         ee_pose.pose.orientation.y = quaternion[1]
#         ee_pose.pose.orientation.z = quaternion[2]
#         ee_pose.pose.orientation.w = quaternion[3]


#         # rospy.loginfo("pose position: {}".format(ee_pose.pose.position))
#         rospy.loginfo("pose orientation: {}".format(ee_pose.pose.orientation))


#         # ee_pose.header.frame_id="base_link"
#         ee_pose.header.frame_id= "head_camera_depth_optical_frame"
#         ee_pose.header.stamp = rospy.Time.now()
#         ee_pose.header.seq = grasp_callback.counter
#         # 

#         # 
    
#         grasp_pub.publish(ee_pose)
# grasp_callback.counter=0


def translate_pose(stampedpose,translation):
    from pyquaternion import Quaternion
    q = Quaternion(stampedpose.pose.orientation.w,stampedpose.pose.orientation.x,stampedpose.pose.orientation.y,stampedpose.pose.orientation.z)
    v = np.array(translation)
    v_rotated = q.rotate(v)
    new_pose = copy.deepcopy(stampedpose)
    new_pose.pose.position.x += v_rotated[0]
    new_pose.pose.position.y += v_rotated[1] 
    new_pose.pose.position.z += v_rotated[2]
    return new_pose


def plan_motion(pose,frame,wait=True):
    move_group.set_pose_target(pose.pose)
    move_group.set_pose_reference_frame(frame)
    move_group.set_planning_time(10)
    rospy.loginfo("pose position: {}".format(pose.pose.position))

    plan = move_group.plan();
    if(len(plan.joint_trajectory.points) == 0):
        rospy.loginfo("No plan found")
        return False

    rospy.loginfo("Moving arm...")

    success = move_group.execute(plan,wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    
    return True



def visualisation_callback(data):
    if(visualisation_callback.running == True):
        return
    if len(data.markers) < 1:
        return
    
    visualisation_callback.running=True

    rospy.loginfo("Number of grasps received: {}".format(len(data.markers)))
    
    grasp = data.markers[2]


    ee_pose = PoseStamped();
    ee_pose.pose = grasp.pose
    ee_pose.pose.position.z 

    ee_pose.header.frame_id= "head_camera_depth_optical_frame"
    ee_pose.header.stamp = rospy.Time.now()
    ee_pose.header.seq = visualisation_callback.counter

    ee_pose = translate_pose(ee_pose,[0,-0.015,0])

    

    visualisation_callback.counter += 1


    tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tranform = tf_buffer.lookup_transform("map",ee_pose.header.frame_id,rospy.Time(0),rospy.Duration(1.0))
    pose_transformed = tf2_geometry_msgs.do_transform_pose(ee_pose,tranform)


    approach_pose = translate_pose(pose_transformed,[-0.1,0,0])    
    grasp_pub.publish(pose_transformed)
    temp_grasp_pub.publish(approach_pose)


    if(~plan_motion(approach_pose,"map",wait=True)):
        visualisation_callback.running=False
        return  
   
    if(~plan_motion(pose_transformed,"map",wait=True)):
        visualisation_callback.running=False
        return

    
    visualisation_callback.running=False    
    

visualisation_callback.running=False
visualisation_callback.counter=0



def createscene():
    rospy.loginfo("creating/updating scene")
    
    scene.addBox("table",0.8,1.6,1,1.8,0,0.2, frame_id="odom")

if __name__ == "__main__":

    #? global variables
    global grasp_pub
    global move_group
    global temp_grasp_pub
    global gripperaction
    global scene

    # Create a node
    rospy.init_node("scfms_demo")
    while not rospy.Time.now():
        pass


    #? Subscribers and publishers
    # rospy.Subscriber("/detect_grasps/clustered_grasps",GraspConfigList,grasp_callback)
    rospy.Subscriber("/detect_grasps/plot_grasps",MarkerArray,visualisation_callback)
    grasp_pub = rospy.Publisher("/scfms/grasp_pose",PoseStamped,queue_size=20)
    temp_grasp_pub = rospy.Publisher("scfms/temp_pose",PoseStamped,queue_size=20)
    
    #? Moveit commander
    move_group = moveit_commander.MoveGroupCommander("arm")
    arm_joints  = move_group.get_joints();
    rospy.loginfo("Joint names: {}".format(arm_joints));
    #? create scene, objects come in while loop to update position during tasks

    scene= PlanningSceneInterface("odom")

    
    # collision_objects = scene.getKnownCollisionObjects()
    # rospy.loginfo("Collision objects: {}".format(collision_objects))

    #? Setup action clients for individual joint control
    move_base = MoveBaseClient()
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    arm_action = FollowTrajectoryClient("arm_controller",arm_joints)
    head_action = PointHeadClient()
    gripper_action = GripperClient()



    #? Setup scene
    rospy.loginfo("looking at the table...")
    head_action.look_at(1.0, 0.0, 0.8, "base_link")
    rospy.loginfo("moving arm...")
    arm_action.move_to([1.571, 0.0, 0.0, -1.571, 0.0, 0.785,0.0])
    rospy.loginfo("closing gripper...")
    gripper_action.close()
    rospy.loginfo("opening gripper...")
    gripper_action.open()
    # rospy.loginfo("Raising torso...")
    # torso_action.move_to([0.4])





    while 1:
        createscene()



    
    # arm_action.move_to([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0])



   

