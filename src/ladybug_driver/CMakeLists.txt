cmake_minimum_required(VERSION 3.0.2)
project(ladybug_driver)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  tf
)

find_package(OpenCV REQUIRED)

# Look for the Ladybug library
find_library(LADYBUG_LIB ladybug HINTS /usr/lib/ladybug)
find_library(LADYBUGVIDEO_LIB ladybugvideo HINTS /usr/lib/ladybug)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ladybug_driver
  /usr/include/ladybug
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ladybug_node src/ladybug_node.cpp src/ladybug_camera.cpp)
target_link_libraries(ladybug_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${LADYBUG_LIB}
  ${LADYBUGVIDEO_LIB}
)

# Install targets
install(TARGETS ladybug_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install include files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)