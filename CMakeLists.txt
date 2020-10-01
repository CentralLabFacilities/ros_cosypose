cmake_minimum_required(VERSION 2.8.3)
project(ros_cosypose)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  object_recognition_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package()
