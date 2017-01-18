cmake_minimum_required(VERSION 2.8.3)
project(sbpl_multimaster)

find_package(catkin REQUIRED COMPONENTS
  rocon_gateway
  rocon_hub
  rospy
  std_msgs
)

catkin_package()

#catkin_install_python(PROGRAMS scripts/generate_multimaster_configs.py
#                     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
)
