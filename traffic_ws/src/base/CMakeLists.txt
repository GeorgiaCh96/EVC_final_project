cmake_minimum_required(VERSION 3.0.2)
project(base)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

# Declare a catkin package
catkin_package()

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
# Install Python scripts
catkin_install_python(PROGRAMS
  src/publisher_node.py
  src/calibration_node.py
  src/decision_node.py
  src/final_subscriber_node_line.py
  src/processing_node_gestures.py
  src/processing_node_line.py
  src/processing_node_motion_det.py
  src/processing_node_barcode.py
  src/processing_node_colour.py
  src/subscriber_node_colour.py
  src/processing_node_yolo.py
  src/view_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
