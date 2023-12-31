cmake_minimum_required(VERSION 3.0.2)
project(amov_gimbal_sdk_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

add_subdirectory(gimbal-sdk)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  sensor_msgs
  image_transport
  cv_bridge
)
##find_package(OpenCV 3.3.1 REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GimbalControl.msg
  GimbalState.msg
  border_coordinates.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CameraControl.srv
  gimbal_control.srv
)



## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amov_gimbal_sdk_ros
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS scripts/yoloV5.py scripts/yoloV5_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/gimbal-sdk/include
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INUCLUDE_DIRS}
)


add_executable(amov_gimbal_node src/amov_gimbal_node.cpp)
add_dependencies(amov_gimbal_node ${${PROJECT_NAME}_EXPORTTED_TARGETS}
                                   ${catkin_EXPORTED_TARGETS}
                                   ${PROJECT_NAME}_generate_messages_cpp
                                   ${PROJECT_NAME}_gencpp)
## Specify libraries to link a library or executable target against
target_link_libraries(amov_gimbal_node
  ${catkin_LIBRARIES}
  serial
  crc32
  gimbal_cmd
)


add_executable(amov_gimbal_image_node src/amov_gimbal_image_node.cpp)
add_dependencies(amov_gimbal_image_node ${${PROJECT_NAME}_EXPORTTED_TARGETS}
                                   ${catkin_EXPORTED_TARGETS}
                                   ${PROJECT_NAME}_generate_messages_cpp
                                   ${PROJECT_NAME}_gencpp)
target_link_libraries(amov_gimbal_image_node 
      ${catkin_LIBRARIES} 
      ${OpenCV_LIBS} 
  )

add_executable(gimbal_control_server src/gimbal_control_server)
add_dependencies(gimbal_control_server amov_gimbal_sdk_ros_gencpp)

target_link_libraries(gimbal_control_server
 ${catkin_LIBRARIES}
 serial
 crc32
 gimbal_cmd
 )
