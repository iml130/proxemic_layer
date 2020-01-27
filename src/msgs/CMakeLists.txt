cmake_minimum_required(VERSION 2.8.3)
project(msgs)

## find other catkin packages
find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
    FILES
    People.msg 
    Person.msg
    Proxemic.msg   
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
