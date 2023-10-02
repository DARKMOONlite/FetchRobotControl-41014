cmake_minimum_required(VERSION 3.7.2)
project(scfms_gazebo)

find_package(catkin)
catkin_package()





install(
  DIRECTORY launch worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
