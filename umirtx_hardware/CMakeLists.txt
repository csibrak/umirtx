cmake_minimum_required(VERSION 2.8.3)
project(umirtx_hardware)

find_package(catkin REQUIRED COMPONENTS controller_manager diagnostic_updater hardware_interface roscpp roslaunch sensor_msgs topic_tools tf geometry_msgs std_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )

add_executable(umirtxnode src/umirtx_base src/umirtx_hardware)
#src/umirtxdiagnostic_updater
target_link_libraries(umirtxnode ${catkin_LIBRARIES})
add_dependencies(umirtxnode umirtxmsgs_gencpp rosserial_msgs_gencpp)
