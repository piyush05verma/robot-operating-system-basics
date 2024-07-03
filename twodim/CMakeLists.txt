cmake_minimum_required(VERSION 3.0.2)
project(twodim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  Matrix2D.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  ## Add this line to include OpenCV headers
)

## Declare a C++ executable
add_executable(twodim_server src/twodim_server.cpp)
add_executable(twodim_client src/twodim_client.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(twodim_server
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}  ## Add this line to link against OpenCV libraries
)

target_link_libraries(twodim_client
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
