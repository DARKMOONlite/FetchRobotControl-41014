


## bag files.


- bag files can be downloaded from the [Intel realsense repo](https://github.com/IntelRealSense/librealsense/blob/master/doc/sample-data.md#files). 
- these can then be run by launching the `depth_image_2_point_cloud.launch` file.

```bash
roslaunch scfms_object_detection depth_image_2_point_cloud.launch bag:="{location of bag file}" 
```

```mermaid
flowchart TB
A(/depth_camera/points) -->|remove far away points| B
A -->|remove things below a certain Y value| B(/no_background/points)

B-->|remove table using ransac via PCL| C(/objects)

C-->|seperate objects via shape consensus|D(/individual objects)
D-->|keep track of objects and create order to pick up in |E(/object_to_pickup)
E-->|GPD to determine poses for the object|F(/pose)


```


#### dealing with point clouds.

currently it seems like 
- X is left and right of the robot.
- z is in front of the robot 
- y is vertical, positive down
- i.e. camera values