cmake_minimum_required(VERSION 3.0.2)
project(ladybug_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
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
  /usr/include/ladybug
)

add_executable(ladybug_node src/ladybug_node.cpp)
target_link_libraries(ladybug_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${LADYBUG_LIB}
  ${LADYBUGVIDEO_LIB}
)

add_executable(test_ladybug src/test_ladybug.cpp)
target_link_libraries(test_ladybug
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
${LADYBUG_LIB}
${LADYBUGVIDEO_LIB}
)