cmake_minimum_required(VERSION 2.8.3)
project(br_echo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        rospy
        std_msgs
        )

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
        CATKIN_DEPENDS rospy std_msgs
        #  DEPENDS system_lib
)

###########
## Build ##
###########

catkin_python_setup()

catkin_install_python(PROGRAMS bin/br_echo DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
