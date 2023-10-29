# Setup Guide

##
- **gazebo** : package dealing with the setup of the gazebo simulator: creating models, etc.
- **moveit**: package dealing with scripts for controlling the fetch robot. 
- **object detection**: deals with point cloud filtering, segmentation and dealing with GPD for grasp pose detection
- **solution**: should contain main files for launching the scfms project. 



## setting up the simulation

currently the method for setting up the simulation is by running 
```bash
roslaunch scfms_moveit demo.launch
```

in another terminal then write to launch the script:
```bash
roslauch scmfs_moveit fetch_commander.launch
```

to mvvoe the fetch robot to the correct position, download the `ros-melodic-teleop-twist-keyboard` and run in another terminal
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```
