cmake_minimum_required(VERSION 2.8.3)
project(motion_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  ros_esccontrol
)

add_service_files(FILES GoStraight.srv)
generate_messages(DEPENDENCIES std_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_esccontrol
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_STANDARD 11)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
   ${catkin_INCLUDE_DIRS}
)

add_executable(motion_controller src/turtle_motion.cpp)
target_link_libraries(motion_controller ${catkin_LIBRARIES})
add_dependencies(motion_controller motion_controller_generate_messages_cpp ros_esccontrol_generate_messages_cpp)

add_executable(motor_test src/turtle_motor_test.cpp)
target_link_libraries(motor_test ${catkin_LIBRARIES})
add_dependencies(motor_test motion_controller_generate_messages_cpp ros_esccontrol_generate_messages_cpp)

