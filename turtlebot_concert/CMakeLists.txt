##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_concert)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED
       COMPONENTS
           roscpp std_srvs
           gateway_msgs
           geometry_msgs
       )
include_directories(SYSTEM include ${catkin_INCLUDE_DIRS})

catkin_package(
       CATKIN_DEPENDS
           roscpp
           std_srvs
           gateway_msgs
           geometry_msgs
       )

##############################################################################
# Project
##############################################################################

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS 
        scripts/teleop.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY solutions
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY concert
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY rapps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY services
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
