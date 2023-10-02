cmake_minimum_required(VERSION 3.7.2)
project(scfms_object_detection)


find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  sensor_msgs
  cmake_modules)
catkin_package()

find_package(Eigen REQUIRED)
find_package(PCL 1.2 REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories()

# add_executable(Corner_Detection src/corner_detector.cpp)
# target_link_libraries(Corner_Detection ${catkin_LIBRARIES})

add_executable(planar_Seperator src/planar_seperator.cpp)
target_link_libraries(planar_Seperator ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
