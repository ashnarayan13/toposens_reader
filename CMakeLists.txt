cmake_minimum_required(VERSION 3.0.2)
project(toposens_task)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  pcl_conversions
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES toposens_task
  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(topo_parser
  src/parser.cpp
)

add_library(topo_cluster
  src/cluster_extraction.cpp
)


# Executable to parse the data file and publish pcl pointcloud
add_executable(${PROJECT_NAME}_node src/publish_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  topo_parser
#  ${PCL_LIBRARIES}
)

# Executable to read pcl pointcloud and publish pole status
add_executable(${PROJECT_NAME}_cluster_node src/publish_cluster_node.cpp)
target_link_libraries(${PROJECT_NAME}_cluster_node
  ${catkin_LIBRARIES}
  topo_cluster
#  ${PCL_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_toposens_task.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

