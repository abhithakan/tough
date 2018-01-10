cmake_minimum_required(VERSION 2.8.3)
project(tough_perception_bringup)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV 2.4 REQUIRED core gpu imgproc calib3d highgui)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        rospy
                                        geometric_shapes
                                        cv_bridge
                                        image_transport
                                        pcl_conversions
                                        pcl_ros
                                        message_filters
                                        multisense_ros
                                        tf_conversions
                                        laser_geometry
                                        laser_assembler
                                        geometry_msgs
                                        sensor_msgs
                                        std_msgs
                                        tough_common
                                        tough_controller_interface
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES tough_perception_bringup
  CATKIN_DEPENDS roscpp cv_bridge image_transport pcl_conversions std_msgs message_filters multisense_ros laser_assembler tough_common tough_controller_interface
  DEPENDS opencv pcl
)

###########
## Build ##
###########
#include_directories(include)
#include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

#add_library(${PROJECT_NAME}  src/left_image_inverter.cpp)
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
#target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
#set_property(
#                TARGET ${PROJECT_NAME}
#   		PROPERTY COMPILE_DEFINITIONS GAZEBO_SIMULATION)

## Declare a C++ executable

# add_executable(left_image_inverter_node src/left_image_inverter_node.cpp)
# add_dependencies(left_image_inverter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(left_image_inverter_node
#   ${catkin_LIBRARIES} ${PROJECT_NAME}
# )

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_val_perception_bringup.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)