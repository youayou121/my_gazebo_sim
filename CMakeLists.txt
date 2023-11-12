cmake_minimum_required(VERSION 3.0.2)
project(my_gazebo_sim)    
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

)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
