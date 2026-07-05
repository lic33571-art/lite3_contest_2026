cmake_minimum_required(VERSION 3.0.2)
project(meter_recognition)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  common
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge common
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/meter_reader.py
  scripts/letter_ocr.py
  scripts/inspection_task.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
