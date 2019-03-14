cmake_minimum_required(VERSION 2.8.3)
project(whycon_ros)

find_package(catkin REQUIRED COMPONENTS
    image_transport
    roscpp
    std_msgs
    geometry_msgs
    message_generation
    dynamic_reconfigure
    cv_bridge
    tf
    )

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/whycon.cfg)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -march=native -ggdb")

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
    FILES
    Marker.msg
    MarkerArray.msg
    )

# Generate services in the 'srv' folder
# add_service_files(
#  FILES
# Charging.srv
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs 
    geometry_msgs 
    )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES charging
    #  CATKIN_DEPENDS image_transport roscpp rospy std_msgs
    CATKIN_DEPENDS message_runtime std_msgs geometry_msgs dynamic_reconfigure
    #  DEPENDS system_lib
    )

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
    )

add_library(whycon_detector src/CCircleDetect.cpp)
add_library(whycon_image src/CRawImage.cpp)
add_library(whycon_timer src/CTimer.cpp)
add_library(whycon_sysmath src/sysmat.cpp)
add_library(whycon_transform src/CTransformation.cpp)
add_library(whycon_necklace src/CNecklace.cpp)
add_library(whycon_gui src/CGui.cpp)

target_link_libraries(whycon_detector whycon_image)

target_link_libraries(whycon_transform
    whycon_sysmath
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

target_link_libraries(whycon_necklace
    whycon_detector
    whycon_transform
    whycon_image
    )

target_link_libraries(whycon_gui
    whycon_image
    whycon_transform
    SDL
    SDL_ttf
    )

add_executable(whycon_ros src/CWhycon.cpp)

add_dependencies(whycon_ros
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    )

target_link_libraries(whycon_ros
    whycon_timer
    whycon_image
    whycon_detector
    whycon_transform
    whycon_gui
    whycon_necklace
    SDL
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

#############
## Install ##
#############

install(TARGETS
    whycon_detector
    whycon_image
    whycon_timer
    whycon_sysmath
    whycon_transform
    whycon_necklace
    whycon_gui
    whycon_ros
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

install(DIRECTORY etc/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/etc
    )

install(DIRECTORY id/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/id
    )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_charging.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

