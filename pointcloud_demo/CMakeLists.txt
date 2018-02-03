cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_demo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp roslaunch topic_tools pcl_ros)
find_package(PCL 1.2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pointcloud_demo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pointcloud_demo src/randompoints.cpp)
target_link_libraries (pointcloud_demo ${PCL_LIBRARIES} ${catkin_LIBRARIES})
