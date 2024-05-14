cmake_minimum_required(VERSION 3.0.2)
project(final_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge image_transport
)

# Install Python scripts properly
catkin_install_python(PROGRAMS hand_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# No add_executable needed for Python scripts
# No target_link_libraries needed for Python scripts
