cmake_minimum_required(VERSION 3.7.2)
project(scfms_moveit)

find_package(catkin)
catkin_package()





install(
  DIRECTORY launch maps config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
