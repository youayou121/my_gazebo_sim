cmake_minimum_required(VERSION 3.0.2)
project(test_mycar)    
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  urdf
  xacro
  gazebo_ros
  gazebo_ros_control
  gazebo_plugins
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_mycar
#  CATKIN_DEPENDS roscpp rospy std_msgs urdf xacro
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(odom_pub_diff src/odom_pub_diff.cpp)
target_link_libraries(odom_pub_diff
  ${catkin_LIBRARIES}
)