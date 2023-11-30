cmake_minimum_required(VERSION 3.0.2)

project(open3d_ros_pointcloud_conversion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
