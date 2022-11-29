cmake_minimum_required(VERSION 3.0.2)
project(optas_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  Load.srv
  ToggleController.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/optas_controller_node.py
  scripts/optas_planner_node.py
  example/example_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
