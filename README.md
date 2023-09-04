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