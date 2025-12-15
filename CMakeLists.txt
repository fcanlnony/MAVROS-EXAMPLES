cmake_minimum_required(VERSION 2.8.3)
project(mavros_examples)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  mavros_msgs
  mavros
  tf
)

catkin_package(
  CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs mavros_msgs mavros tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/circle_trajectory.py
  scripts/circle_trajectory_wind_resist.py
  scripts/rectangle_trajectory.py
  scripts/rectangle_trajectory_wind_resist.py
  scripts/cylinder_trajectory.py
  scripts/cylinder_trajectory_wind_resist.py
  scripts/figure8_trajectory.py
  scripts/figure8_trajectory_wind_resist.py
  scripts/helix_trajectory.py
  scripts/helix_trajectory_wind_resist.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
