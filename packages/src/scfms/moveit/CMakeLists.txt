cmake_minimum_required(VERSION 3.7.2)
project(scfms_moveit)

find_package(catkin)
catkin_package()


install(FILES
  launch/movegroup.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


catkin_install_python(PROGRAMS src/movegroup.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(
  DIRECTORY launch maps config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
