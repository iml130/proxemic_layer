cmake_minimum_required(VERSION 2.8.3)
project(people_publisher)

## find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  msgs
  roscpp
  rospy
  std_msgs
  tf
  dynamic_reconfigure
  costmap_2d
)


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/people_publisher.cfg
)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(people_publisher src/people_publisher.cpp)
target_link_libraries(people_publisher ${catkin_LIBRARIES})
add_dependencies(people_publisher ${PROJECT_NAME}_gencfg)
