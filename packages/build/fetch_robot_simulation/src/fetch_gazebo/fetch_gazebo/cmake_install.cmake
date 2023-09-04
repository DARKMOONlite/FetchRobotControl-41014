# Install script for directory: /home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/src/fetch_gazebo/fetch_gazebo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/sebastian/git/FetchRobotControl-41014/packages/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sebastian/git/FetchRobotControl-41014/packages/build/fetch_robot_simulation/src/fetch_gazebo/fetch_gazebo/catkin_generated/installspace/fetch_gazebo.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetch_gazebo/cmake" TYPE FILE FILES
    "/home/sebastian/git/FetchRobotControl-41014/packages/build/fetch_robot_simulation/src/fetch_gazebo/fetch_gazebo/catkin_generated/installspace/fetch_gazeboConfig.cmake"
    "/home/sebastian/git/FetchRobotControl-41014/packages/build/fetch_robot_simulation/src/fetch_gazebo/fetch_gazebo/catkin_generated/installspace/fetch_gazeboConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetch_gazebo" TYPE FILE FILES "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/src/fetch_gazebo/fetch_gazebo/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfetch_gazebo_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfetch_gazebo_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfetch_gazebo_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/sebastian/git/FetchRobotControl-41014/packages/devel/lib/libfetch_gazebo_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfetch_gazebo_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfetch_gazebo_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfetch_gazebo_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfetch_gazebo_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/fetch_gazebo" TYPE DIRECTORY FILES "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/src/fetch_gazebo/fetch_gazebo/include/fetch_gazebo/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/fetch_gazebo" TYPE PROGRAM FILES
    "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/src/fetch_gazebo/fetch_gazebo/scripts/prepare_simulated_robot.py"
    "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/src/fetch_gazebo/fetch_gazebo/scripts/prepare_simulated_robot_pick_place.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetch_gazebo" TYPE DIRECTORY FILES
    "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/src/fetch_gazebo/fetch_gazebo/config"
    "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/src/fetch_gazebo/fetch_gazebo/include"
    "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/src/fetch_gazebo/fetch_gazebo/launch"
    "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/src/fetch_gazebo/fetch_gazebo/robots"
    "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/src/fetch_gazebo/fetch_gazebo/worlds"
    "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/src/fetch_gazebo/fetch_gazebo/models"
    )
endif()

