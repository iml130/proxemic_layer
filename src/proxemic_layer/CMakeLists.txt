cmake_minimum_required(VERSION 2.8.3)
project(proxemic_layer)

## find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  msgs
  roscpp
  tf
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
    cfg/proxemic.cfg
 )

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS costmap_2d dynamic_reconfigure roscpp msgs)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${catkin_INCLUDE_DIRS} include)
add_library(proxemic_layer src/proxemic_layer.cpp)
add_dependencies(proxemic_layer ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
