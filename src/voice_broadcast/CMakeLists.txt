cmake_minimum_required(VERSION 3.0.2)
project(voice_broadcast)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  common
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs common
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/tts_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
