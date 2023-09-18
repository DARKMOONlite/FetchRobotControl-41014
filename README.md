# FetchRobotControl-41014
controlling a fetch robotic arm via visual servoing in ROS1 Melodic

Contributors: Angus Cheng, Sebastian Schroder, Zahne Simon



## Installation:
1. follow the [melodic setup guide](http://wiki.ros.org/melodic/Installation/Ubuntu)

2. clone the repo 
```bash
    git clone https://github.com/DARKMOONlite/FetchRobotControl-41014.git
```

3. install the required dependencies:
```bash
    sudo apt install ros-melodic-fetch-calibration ros-melodic-fetch-open-auto-dock \
ros-melodic-fetch-navigation ros-melodic-fetch-tools  ros-melodic-robot-controllers ros-melodic-rgbd-launch ros-melodic-moveit-core -y
```
4. build the packages
```bash
    cd FetchRobotControl-41014/packages
    catkin_make
    source devel/setup.bash
```
5. test that it works by running a simple gazebo simulation
```bash
    roslaunch fetch_gazebo simulation.launch
```
6. Download the realsense2 camera packages. [link to relavent question](https://answers.ros.org/question/364033/realsense2_camera-cannot-locate-rosdep-definition-for-librealsense2/)
```bash
    sudo apt-get install ros-melodic-realsense2-camera ros-melodic-realsense2-description ros-melodic-librealsense2 
```
7. install other necessary packages.
```bash
    sudo apt-get install ros-melodic-moveit ros-melodic-simple-grasping ros-melodic-moveit-visual-tools ros-melodic-rosparam-shortcuts
```



### Demos
- **here's a list of interesting demos, that you should check work on your machine. if any packages are missing, please add them to the aboce package list.**

    - ```roslaunch fetch_moveit_config demo.launch ``` *[moveit demo](https://docs.fetchrobotics.com/manipulation.html)*
    - ```roslaunch fetch_gazebo simulation.launch``` *[gazebo demo](https://docs.fetchrobotics.com/gazebo.html)*
    - ```roslaunch fetch_gazebo playground.launch``` & ```roslaunch fetch_gazebo_demo demo.launch```  *[full gazebo + sensor demo](https://docs.fetchrobotics.com/gazebo.html#mm-demo)*



### Tips:

- information about the fetch robot and using it can be found at [Fetch robotics website](https://docs.fetchrobotics.com/index.html)