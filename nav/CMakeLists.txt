cmake_minimum_required(VERSION 2.8.3)
project(nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  nav_msgs
  geometry_msgs
  visualization_msgs
  genmsg
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(map_path src/map_path.cpp)
target_link_libraries(map_path ${catkin_LIBRARIES})

add_executable(map src/map.cpp)
target_link_libraries(map ${catkin_LIBRARIES})

add_executable(map_image_loader src/map_image_loader.cpp)
target_link_libraries(map_image_loader ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)

## Declare a C++ executable for path visualization node
add_executable(map_vis src/map_vis.cpp)
## Add dependencies
add_dependencies(map_vis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link executable against
target_link_libraries(map_vis ${catkin_LIBRARIES})
