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

add_executable(planar_separator src/planar_separator.cpp)
target_include_directories(planar_separator PUBLIC include)
target_link_libraries(planar_separator ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(border_extractor src/border_extractor.cpp)
target_include_directories(border_extractor PUBLIC include)
target_link_libraries(border_extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(background_separator src/background_separator.cpp)
target_include_directories(background_separator PUBLIC include)
target_link_libraries(background_separator ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
