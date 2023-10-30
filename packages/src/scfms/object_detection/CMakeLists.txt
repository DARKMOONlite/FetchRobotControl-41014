cmake_minimum_required(VERSION 3.7.2)
project(scfms_object_detection)


find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  sensor_msgs
  gpd_ros
  cmake_modules)
catkin_package()

find_package(Eigen REQUIRED)
find_package(PCL 1.2 REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories()

set(CMAKE_CXX_STANDARD 17)

# add_executable(Corner_Detection src/corner_detector.cpp)
# target_link_libraries(Corner_Detection ${catkin_LIBRARIES})

add_executable(planar_separator src/planar_separator.cpp)
target_include_directories(planar_separator PUBLIC include)
target_link_libraries(planar_separator ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(planar_separator ${catkin_EXPORTED_TARGETS})


# add_executable(border_extractor src/border_extractor.cpp)
# target_include_directories(border_extractor PUBLIC include)
# target_link_libraries(border_extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# add_dependencies(border_extractor ${catkin_EXPORTED_TARGETS})


add_executable(background_separator src/background_separator.cpp)
target_include_directories(background_separator PUBLIC include)
target_link_libraries(background_separator ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(background_separator ${catkin_EXPORTED_TARGETS})


add_executable(cluster_extractor src/cluster_extractor.cpp)
target_include_directories(cluster_extractor PUBLIC include)
target_link_libraries(cluster_extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(cluster_extractor ${catkin_EXPORTED_TARGETS})


# add_executable(grasp_picker src/grasp_picker.cpp)
# target_include_directories(grasp_picker PUBLIC include)
# target_link_libraries(grasp_picker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# add_dependencies(grasp_picker ${catkin_EXPORTED_TARGETS})


install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
