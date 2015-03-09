cmake_minimum_required(VERSION 2.8.3)
project(roah_devices)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  message_generation
  std_msgs
  std_srvs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

find_package(Qt4 REQUIRED QtCore QtGui)
include(${QT_USE_FILE})


################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
file(GLOB MESSAGE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg)
add_message_files(FILES ${MESSAGE_FILES})

## Generate services in the 'srv' folder
file(GLOB SERVICE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/srv ${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv)
add_service_files(FILES ${SERVICE_FILES})

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
)


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
INCLUDE_DIRS include
LIBRARIES rqt_roah_devices
CATKIN_DEPENDS message_runtime std_msgs std_srvs # roscpp
#  DEPENDS system_lib
)


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(gen/)
add_dependencies(roah_devices_qt roah_devices_generate_messages_cpp)

include_directories(
  ${INCLUDE_DIRS}
)

add_executable(roah_devices src/roah_devices.cpp)
add_dependencies(roah_devices roah_devices_generate_messages_cpp)
target_link_libraries(roah_devices ${catkin_LIBRARIES})

add_executable(dummy_server src/dummy_server.cpp)
target_link_libraries(dummy_server ${catkin_LIBRARIES})

add_library(rqt_roah_devices src/devices_control.cpp)
add_dependencies(rqt_roah_devices roah_devices_generate_messages_cpp)
target_link_libraries(rqt_roah_devices roah_devices_qt ${QT_LIBRARIES} ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
file(GLOB INSTALL_SCRIPTS ${PROJECT_SOURCE_DIR}/scripts/*)
install(PROGRAMS ${INSTALL_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS roah_devices rqt_roah_devices
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
install(FILES include/topic_receiver.h
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
file(GLOB LAUNCH_FILES ${PROJECT_SOURCE_DIR}/launch/*.launch)
install(FILES ${LAUNCH_FILES}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_roah_devices.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
