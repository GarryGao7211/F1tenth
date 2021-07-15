cmake_minimum_required(VERSION 3.0.2)
project(yifei_roslab)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
add_compile_options(-std=c++11)
set (CMAKE_CXX_FLAGS -O3)
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS_RELEASE "-O3")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  roslaunch
  nav_msgs
  ackermann_msgs
  geometry_msgs
  visualization_msgs
  tf
)

find_package(Eigen3 REQUIRED)
#roslaunch_add_file_check(launch)


################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
 #add_message_files(
 #  FILES scan_range.msg
 #)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES AddTwoInts.srv
 #)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 #generate_messages(
  # DEPENDENCIES
  # std_msgs
#)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES yifei_roslab
  LIBRARIES rrt
  CATKIN_DEPENDS ackermann_msgs geometry_msgs roscpp rospy sensor_msgs std_msgs tf
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
 ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/yifei_roslab.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(talker src/talker.cpp)

#add_dependencies(talker yifei_roslab_generate_messages_cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(talker ${catkin_LIBRARIES})

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener yifei_roslab_generate_messages_cpp)

#add_executable(lidar_processing src/lidar_processing.cpp)
#target_link_libraries(lidar_processing ${catkin_LIBRARIES})
#add_dependencies(lidar_processing yifei_roslab_generate_messages_cpp)


#add_executable(safety_node src/safety_node.cpp)

#target_link_libraries(safety_node
#  ${catkin_LIBRARIES}
#)

#add_executable(rrt_node node/rrt_node.cpp src/rrt.cpp)
#add_dependencies(rrt_node ${f110_rrt_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(wall_follow src/wall_follow.cpp)

#target_link_libraries(wall_follow
 # ${catkin_LIBRARIES}
#)

add_executable(pure_pursuit src/pure_pursuit.cpp)

target_link_libraries(pure_pursuit
  ${catkin_LIBRARIES}
)
#add_executable(reactive_gap_follow src/reactive_gap_follow.cpp)

#target_link_libraries(reactive_gap_follow
#  ${catkin_LIBRARIES}
#)

#add_executable(student_scan_matcher src/scan_match.cpp src/transform.cpp src/correspond.cpp src/visualization.cpp)
#target_link_libraries(student_scan_matcher ${catkin_LIBRARIES})

